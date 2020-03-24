/*
 *  lrf_node_rf.c : <Write brief>
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

//Includes
#include "lrf_node_rf.h"
#include "common_util.h"
#include "ms_timer.h"
#include "KXTJ3.h"
#include "aa_aaa_battery_check.h"
#include "random_num.h"
#include "hal_gpio.h"
#include "string.h"
#include "log.h"
#include "hal_nop_delay.h"
#include "gps_mod.h"

//macros
/** MS Timer used by this module */
#define MOD_TIMER CONCAT_2(MS_TIMER, MS_TIMER_USED_LRF_NODE_MOD)
#define S_to_MS(x)  ( 1000* (x))
/** Accelerometer frequency in sensing state */
#define SENSE_FREQ_S (1)
#define SENSE_FREQ_MS (S_to_MS(SENSE_FREQ_S))
#define SENSE_FREQ_MIN_DEV_S 0
#define SENSE_FREQ_MAX_DEV_S 1
/** Accelerometer frequency in tilted state */
#define TILT_FREQ_S (5)
#define TILT_FREQ_MS (S_to_MS(TILT_FREQ_S))
#define TILT_FREQ_MIN_DEV_S 0
#define TILT_FREQ_MAX_DEV_S 3
/** Accelerometer frequency in maintain state */
#define MAINTAIN_FREQ_S (30 * 60)
#define MAINTAIN_FREQ_MS (S_to_MS(MAINTAIN_FREQ_S))
#define MAINTAIN_FREQ_MIN_DEV_S 0
#define MAINTAIN_FREQ_MAX_DEV_S 3
/** Alive signal frequency. Alive signal is to be sent if no other package has been sent */
#define ALIVE_FREQ_S (30 * 60)
#define ALIVE_FREQ_MS (S_to_MS(ALIVE_FREQ_S))


#define GPS_TIMEOUT_S  (300)
#define GPS_TIMEOUT_MS (S_to_MS(GPS_TIMEOUT_S))

#define NO_RF_PKTS 5

/** Maximum length of RF packet */
#define RF_MAX_LEN (16)

/** PKT Structure */
typedef struct
{
    uint8_t batt_volt;
    uint8_t angle;
}node_rf_pkt_t;

//typedefs
typedef enum
{
    PKT_ALIVE,
    PKT_SENSE,
    PKT_MAINTAIN,
    PKT_INVALID,
}pkt_types_t;

typedef enum 
{
    STATE_SLEEP,
    STATE_RUN,
    STATE_RF,
    STATE_INVALID,
}node_states_t;


//unchanging variables

//global variables
static uint32_t g_pkt_cnt = 0;
/** Variable to keep track of  */
static node_states_t g_node_state;
/** Variable to keep track of packet type to  be sent */
static pkt_types_t g_current_pkt_type = PKT_ALIVE;
/**Variable to store TCXO pin number */
static uint32_t g_pin_tcxo_en;
/***/
static uint32_t g_sleep_freq_ticks;
/***/
static uint32_t g_gps_timeout_ms = GPS_TIMEOUT_MS;
/**variable to store RF HW information*/

void node_rf_tx_done (uint32_t status);

void node_rf_tx_failed (uint32_t status);

void node_gps_timeout ();

void node_gps_loc_update (gps_mod_loc_t * gps);

static rf_comm_radio_t g_rf_comm_radio = 
{
    .rf_tx_done_handler = node_rf_tx_done,
    .rf_tx_failed_handler = node_rf_tx_failed,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
};

static gps_mod_config_t g_gps_config;

static rf_comm_hw_t g_rf_comm_hw;

//function declaration

/**
 * @brief Function to wakeup radio from sleep
 */
void node_rf_wakeup ();

/**
 * @brief Function to put radio to sleep
 */
void node_rf_sleep ();

/**
 * @brief Function to measure angle from reference axis
 * @param acce_comp Acceleration component along reference axis
 * @return Angle from reference axis
 */
uint8_t get_angle (int32_t acce_comp);

/**
 * @brief Function to check if net acceleration in below threshold level.
 * @param threshold Threshold on net acceleration (in milli g)
 * @return Node's motion status
 * @retval 1 if acceleration is greater than or equal to threshold
 * @retval 0 if acceleration is less than threshold value 
 */
bool is_node_moving (uint32_t threshold);

/** Common timer handler. For each state change time duration will get updated */
void ms_timer_handler (void);

void state_sleep (void);
void state_run (void);
void state_rf (void);

void node_gps_timeout ()
{
    g_node_state = STATE_RF;
    gps_mod_stop ();
}

void node_gps_loc_update (gps_mod_loc_t * gps)
{
    g_node_state = STATE_RF;
    gps_mod_stop ();
    log_printf ("Lat %d Lng %d\n", gps->lat, gps->lng);
}


/**
 * @brief Function to manage switching between states.
 */
void switch_manage ();

//function array

//function definitions

void node_rf_wakeup ()
{
    hal_gpio_pin_set (g_pin_tcxo_en);
    hal_nop_delay_ms (50);
    rf_comm_wake();
    log_printf("Radio ID :0x%x\n", rf_comm_get_radio_id ());
    rf_comm_radio_init (&g_rf_comm_radio, &g_rf_comm_hw);

    rf_comm_idle ();
    rf_comm_enable_irq ();
}

void node_rf_sleep ()
{
    log_printf("R Sl\n");
    rf_comm_flush ();
    rf_comm_sleep ();
//    hal_nop_delay_ms (5);
    hal_gpio_pin_clear (g_pin_tcxo_en);
    rf_comm_disable_irq ();
}

void state_sleep (void)
{
    gps_mod_stop ();
}

void state_run (void)
{
    gps_mod_start (g_gps_timeout_ms);
}

void state_rf (void)
{
    g_pkt_cnt = 0;
    uint32_t l_timer_duration = (random_num_generate (8, 16) * 500);
    ms_timer_start (MOD_TIMER, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(l_timer_duration),
                    ms_timer_handler);
}

void switch_manage ()
{
    switch (g_node_state)
    {
        case STATE_SLEEP : 
            state_sleep ();
            break;
            
        case STATE_RUN : 
            state_run ();
            break;
            
        case STATE_RF : 
            state_rf ();
            break;
            
        case STATE_INVALID : 
//            state_sleep ();
            break;
            
    }
}

void ms_timer_handler (void)
{
    uint8_t l_pkt_data[9];
    l_pkt_data[0] = aa_aaa_battery_status ();
    memcpy (&l_pkt_data[1], gps_mod_get_last_location (), sizeof(gps_mod_loc_t));
    node_rf_wakeup ();
    rf_comm_pkt_send (g_current_pkt_type, (uint8_t *)&l_pkt_data, 9);
    log_printf("L %d %d C %d\n", gps_mod_get_last_location ()->lat,
               gps_mod_get_last_location ()->lng, g_pkt_cnt);
    g_pkt_cnt++;
    if(g_pkt_cnt < NO_RF_PKTS)
    {
        uint32_t l_nxt_pkt = (random_num_generate (8, 16) * 500);
        ms_timer_start (MOD_TIMER, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(l_nxt_pkt),
                        ms_timer_handler);
        
    }
    else
    {
        g_node_state = STATE_SLEEP;
        g_pkt_cnt = 0;
    }
}

void node_rf_tx_done (uint32_t status)
{
    log_printf("Tx Done : %d\n", status);
    node_rf_sleep ();
}

void node_rf_tx_failed (uint32_t status)
{
    //in certain cases
    log_printf("Tx Failed : %d\n", status);
    {
        node_rf_sleep ();

    }
}

/** APIs */
void lrf_node_mod_init (lrf_node_mod_init_t * p_mod_init)
{
    //initialize RF comm module
    g_rf_comm_radio.bitrate = p_mod_init->radio_params.bitrate;
    g_rf_comm_radio.center_freq = p_mod_init->radio_params.center_freq;
    g_rf_comm_radio.freq_dev = p_mod_init->radio_params.fdev;
    g_rf_comm_radio.tx_power = p_mod_init->radio_params.tx_power;
    
    memcpy (&g_rf_comm_hw, &p_mod_init->radio_gpio, sizeof(rf_comm_hw_t));
    
    rf_spi_init ((rf_spi_init_t * )&p_mod_init->radio_spi);
    //Save RF packet header
    rf_comm_pkt_t l_rf_head = 
    {
        .max_len = RF_MAX_LEN,
        .app_id = p_mod_init->radio_header.app_id,
        .dev_id = p_mod_init->radio_header.prod_id,
    };
    rf_comm_pkt_config (&l_rf_head);
    
    g_gps_config.baudrate = p_mod_init->gps.baudrate;
    g_gps_config.en_pin = p_mod_init->gps.gps_en_pin;
    g_gps_config.comm_timeout_irq_priority =
        p_mod_init->gps.timeout_irq_priority;
    g_gps_config.comm_running_irq_priority = 
        p_mod_init->gps.running_irq_priority;
    g_gps_config.resolution = GPS_MOD_RES_D6;
    g_gps_config.loc_handler = node_gps_loc_update;
    g_gps_config.timeout_handler = node_gps_timeout;
    log_printf ("%s\n", __func__);
    
    hal_nop_delay_ms (500); 
    gps_mod_init (&g_gps_config);
    
    g_sleep_freq_ticks = MS_TIMER_TICKS_MS(p_mod_init->sleep_ms);
        
    g_pin_tcxo_en = p_mod_init->rf_tcxo_pin;
    hal_gpio_cfg_output (g_pin_tcxo_en, 0);
    
    //
}

void lrf_node_mod_start ()
{
    //start sensing the angle
    g_node_state = STATE_RUN;
    g_pkt_cnt = 0;
    node_rf_wakeup ();
    node_rf_sleep ();
    gps_mod_start (g_gps_timeout_ms);

}

void lrf_node_mod_stop ()
{
    //stop everything except accelerometer
    ms_timer_stop (MOD_TIMER);
    g_node_state = STATE_INVALID;
    g_pkt_cnt = 0;
    hal_gpio_pin_set (g_pin_tcxo_en);
    node_rf_sleep ();
}

void lrf_node_mod_update_rf_head (lrf_node_mod_rf_head_t * p_head)
{
    rf_comm_pkt_t l_rf_pkt = 
    {
        .max_len = RF_MAX_LEN,
        .dev_id = p_head->prod_id,
        .app_id = p_head->app_id,
    };
    rf_comm_pkt_config (&l_rf_pkt);
}

void lrf_node_mod_update_rf_params (lrf_node_mod_rf_params_t * p_params)
{
    //reinitialize RF comm with new params
    g_rf_comm_radio.bitrate = p_params->bitrate;
    g_rf_comm_radio.center_freq = p_params->center_freq;
    g_rf_comm_radio.freq_dev = p_params->fdev;
    g_rf_comm_radio.tx_power = p_params->tx_power;
}


void lrf_node_mod_add_ticks (uint32_t ticks)
{
    static uint32_t sleep_ticks;
    sleep_ticks += ticks;
    log_printf("T %d S %d\n", sleep_ticks, g_node_state);
    static node_states_t l_prev_state = STATE_RUN;
    if (l_prev_state != g_node_state)
    {
        l_prev_state = g_node_state;
        switch_manage ();
    }
    switch (g_node_state)
    {
        case STATE_SLEEP : 
        {
            if(sleep_ticks > g_sleep_freq_ticks)
            {
                g_node_state = STATE_RUN;
//                sleep_ticks = 0;
            }
            break;
        }
        case STATE_RUN :
        {
            gps_mod_add_ticks (ticks);
            break;
        }
        case STATE_RF :
        {
            break;
        }
        case STATE_INVALID :
        {
            break;
        }
    }
    if(sleep_ticks > g_sleep_freq_ticks)
    {
        sleep_ticks= 0;
    }

}


void lrf_node_mod_gps_only (bool state)
{
    if(state)
    {
        gps_mod_always_on ();
    }
    else
    {
        gps_mod_stop ();
    }
}

gps_mod_loc_t * lrf_node_mod_get_current_loc ()
{
    return gps_mod_get_last_location ();
}


void lrf_node_mod_rf_check ()
{
    node_rf_wakeup ();
    node_rf_sleep ();
    while(1)
    {
        __WFI ();
    }
    
}
