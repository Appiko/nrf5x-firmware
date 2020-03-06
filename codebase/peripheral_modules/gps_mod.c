/*
 *  gps_mod.c : <Write brief>
 *  Copyright (C) 2019  Appiko
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "gps_mod.h"
#include "hal_gpio.h"
#include "ms_timer.h"
#include "nrf_assert.h"
#include "string.h"
#include "minmea.h"
#include "log.h"
#include "stdbool.h"

#define MAX_LINE_BUFF_LEN 81

#define MAX_HDOP    3

#define MIN_FIXES 30

typedef struct 
{
    struct minmea_float lat;
    struct minmea_float lng;
    struct minmea_float hdop;
}gps_data_t;

static gps_data_t gps_data;

static uint32_t g_timeout_ticks;

volatile uint32_t g_current_ticks;

static uint32_t g_en_pin;

static bool g_is_always_on = false;

static hal_uart_baud_t g_baudrate;

static app_irq_priority_t g_timeout_irq_priority;

static app_irq_priority_t g_running_irq_priority;

static gps_mod_resolution_t g_loc_res;

static gps_mod_loc_t g_current_loc;

//static gps_mod_loc_t g_last_valid_loc;

uint8_t g_line[MAX_LINE_BUFF_LEN];

void (* g_loc_handler) (gps_mod_loc_t * loc);

void (* g_timeout_handler) ();


static uint32_t g_fix_cnt = 0;

void validate_gps_data ()
{
    if(((float)(gps_data.hdop.value/gps_data.hdop.scale) < MAX_HDOP) &&
        (gps_data.lat.value != 0) && (gps_data.lng.value != 0))
    {
        g_fix_cnt++;
        
        g_current_loc.lng = (uint32_t)((float)(minmea_tocoord(&gps_data.lng)) *
                                g_loc_res);
        g_current_loc.lat = (uint32_t)((float)(minmea_tocoord(&gps_data.lat)) *
                                g_loc_res);
        if((g_fix_cnt > MIN_FIXES)(g_current_loc.lng != (-1)) && (g_is_always_on == false))
        {
            g_loc_handler(&g_current_loc);
            g_fix_cnt = 0;
        }
    }
}


void update_location ()
{
    switch (minmea_sentence_id((char *)g_line, false))
    {
        case MINMEA_SENTENCE_GLL:
        {
            struct minmea_sentence_gll frame;
            if (minmea_parse_gll(&frame, (char *)g_line)) 
            {
//                                    log_printf("GLL Coordinates: (%d/%d,%d/%d)\n\n",
//                                            frame.latitude.value, frame.latitude.scale,
//                                            frame.longitude.value, frame.longitude.scale
//                                            );
                gps_data.lat = frame.latitude;
                gps_data.lng = frame.longitude;

            } else {
//                                    log_printf(" GLL Err. \n");
            }
        }

        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, (char *)g_line)) 
            {
//                log_printf("$GGA: LatLng: (%d/%d,%d/%d)\n Accuracy: (%d/%d) \n\n",
//
//                        frame.latitude.value, frame.latitude.scale,
//                        frame.longitude.value, frame.longitude.scale,
//                        frame.hdop.value, frame.hdop.scale
//
//                        );

                gps_data.hdop.scale = frame.hdop.scale;
                gps_data.lat.scale = frame.latitude.scale;
                gps_data.lng.scale = frame.longitude.scale;
                
                gps_data.hdop.value = frame.hdop.value;
                gps_data.lat.value = frame.latitude.value;
                gps_data.lng.value = frame.longitude.value;
                validate_gps_data ();
                
            }
            else
            {
//                                    log_printf("GGA Err.\n");
            }
        }
            break;
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, (char *)g_line))
            {
//                                    log_printf("$RMC: LatLng: (%d/%d, %d/%d)\n\n",
//                                            frame.latitude.value, frame.latitude.scale,
//                                            frame.longitude.value, frame.longitude.scale
//                                            );
//
                gps_data.lat = frame.latitude;
                gps_data.lng = frame.longitude;
            }
            else
            {
            }

        }
            break;
        default:
            break;

    }
}

void rx_data_parser(uint8_t byte)
{
    static uint32_t line_length = 0;
    if (byte == '\n' || byte == '\r')
    {
        update_location ();
        line_length = 0;
    }
    else if (line_length == 0 && byte == '$') 
    {
        // push G to (char *)g_line at index 0
        memset(g_line,'\0',MAX_LINE_BUFF_LEN);
        g_line[line_length++] = '$';
    } 
    else if (line_length != 0) 
    {
        //push to (char *)g_line
        g_line[line_length++] = byte;
    }
}


void gps_mod_init (gps_mod_config_t * p_mod_init)
{
    ASSERT(p_mod_init->timeout_handler!=NULL);
    ASSERT(p_mod_init->loc_handler!=NULL);
    g_timeout_handler = p_mod_init->timeout_handler;
    g_loc_handler = p_mod_init->loc_handler;
    g_loc_res = p_mod_init->resolution;
    g_baudrate = p_mod_init->baudrate;
    g_timeout_irq_priority = p_mod_init->comm_timeout_irq_priority;
    g_running_irq_priority = p_mod_init->comm_running_irq_priority;
    hal_uarte_uninit ();
    hal_uarte_init(g_baudrate, g_timeout_irq_priority);
    g_en_pin = p_mod_init->en_pin;
    hal_gpio_cfg_output (g_en_pin, 1);
    hal_gpio_pin_set (g_en_pin);
    
}

void gps_mod_start (uint32_t timeout_ms)
{
    log_printf("GPS Start\n");
    hal_gpio_pin_clear (g_en_pin);
    hal_uarte_init(g_baudrate, g_timeout_irq_priority);
    g_is_always_on = false;
    g_current_loc.lat = 0;
    g_current_loc.lng = 0;
    g_timeout_ticks = MS_TIMER_TICKS_MS (timeout_ms);
    g_current_ticks = 0;
    hal_uarte_start_rx (rx_data_parser);
}

void gps_mod_always_on ()
{
    hal_uarte_init(g_baudrate, g_running_irq_priority);
    g_is_always_on = true;
    hal_gpio_pin_clear (g_en_pin);
    g_current_loc.lat = 0;
    g_current_loc.lng = 0;
    hal_uarte_start_rx (rx_data_parser);
}


void gps_mod_add_ticks (uint32_t ticks)
{
    g_current_ticks += ticks;
    log_printf("G %d %d\n", g_current_ticks, g_timeout_ticks);
    if ((g_current_ticks>g_timeout_ticks) && (g_is_always_on == false))
    {
        hal_uarte_stop_rx ();
        hal_uarte_uninit ();
        g_timeout_handler ();
        hal_gpio_pin_set (g_en_pin);
        g_fix_cnt = 0;
    }
}

void gps_mod_stop ()
{
    hal_gpio_pin_set (g_en_pin);
    g_is_always_on = false;
    hal_uarte_stop_rx ();
    hal_uarte_uninit ();
    g_current_ticks = 0;
}

gps_mod_loc_t * gps_mod_get_last_location ()
{
    return &g_current_loc;
}

void gps_mod_process ()
{
    hal_uarte_process ();
}