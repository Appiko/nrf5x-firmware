/* 
 * File:   sensebe_rx_detect.c
 * Copyright (c) 2018 Appiko
 * Created on 29 October, 2018, 12:22 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */

#include "hal_gpio.h"
#include "ms_timer.h"
#include "sensebe_rx_rev1.h"
#include "sensebe_rx_detect.h"
#include "log.h"
#include "led_ui.h"
#include "led_seq.h"
#include "tssp_detect.h"
#include "device_tick.h"
#include "cam_trigger.h"
#include "simple_adc.h"

typedef enum
{
    MOTION_ONLY,
    TIMER_ONLY,
    MOTION_TIMER_BOTH,
    MAX_TRIGGERS
    
}trigger_t;

typedef enum
{
    MOTION_FEEDBACK,
    MOTION_WAIT_FOR_TIMEOUT,
    MOTION_SYNC,
    MOTION_IDLE,
}motion_detection_states_t;

/** The fast tick interval in ms in the Sense mode */
#define SENSE_FAST_TICK_INTERVAL_MS      1000
/** The slow tick interval in ms in the Sense mode */
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

#define CAMERA_TIMEOUT_30S  MS_TIMER_TICKS_MS(30000)

#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(600000)

#define LIGHT_THRESHOLD_MULTIPLING_FACTOR 32

static motion_detection_states_t state = MOTION_IDLE;

static uint32_t detect_time_pass = 0;

static uint32_t wait_window_timepassed = 0;

static uint32_t light_intensity_pin;

void timer_1s_handler (void);
void timer_200ms_handler (void);

void timer_200ms_handler (void)
{
    tssp_detect_stop ();
    ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(1000),
                    timer_1s_handler);
}

void timer_1s_handler (void)
{
    tssp_detect_pulse_detect ();
    ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(200),
                    timer_200ms_handler);
}

void camera_unit_handler(uint32_t trigger)
{
    log_printf("%s\n", __func__);
    log_printf("State : %d\n", trigger);
    switch(trigger)
    {
        case MOTION_ONLY : 
            if (state != MOTION_FEEDBACK)  
            {
                device_tick_process ();
            }
            if(state != MOTION_SYNC)
            {
               tssp_detect_window_detect ();
            }
            break;
        case TIMER_ONLY :
            break;
    }
    
}

bool light_sense (uint8_t light_threshold_mode)
{
    bool mode;
    uint8_t light_threshold;
    mode = light_threshold_mode & 0x1;
    light_threshold = ((light_threshold_mode & 0xFE)>>1) 
        * LIGHT_THRESHOLD_MULTIPLING_FACTOR;
    uint32_t light_intensity;
    light_intensity = simple_adc_get_value (SIMPLE_ADC_GAIN1_6, 
                                            light_intensity_pin);
    if(((mode == 1) && (light_intensity >= light_threshold)) ||
       ((mode == 0) && (light_intensity <= light_threshold)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void timer_trigger_handler ()
{
    
    log_printf("%s\n", __func__);
    if(cam_trigger_is_on () == false)
    {
        cam_trigger (TIMER_ONLY);
    }
}
void window_trigger ()
{
    log_printf("%s\n", __func__);
    {
        tssp_detect_stop ();
        if(state == MOTION_FEEDBACK)
        {
            led_ui_single_start (LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
            wait_window_timepassed = 0;
        }
        else 
        {
            device_tick_switch_mode (DEVICE_TICK_FAST);
            tssp_detect_pulse_detect ();
            state = MOTION_WAIT_FOR_TIMEOUT;
        }
        cam_trigger (MOTION_ONLY);
    }
}

void pulse_detect_handler ()
{
    if(state == MOTION_SYNC)
    {
        detect_time_pass = 0;
        ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
        tssp_detect_window_detect ();
    }
    else
    {
        device_tick_process ();
        wait_window_timepassed = 0;
    }
    state = MOTION_IDLE;
}

void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);
    
    light_intensity_pin = sensebe_rx_detect_config->photodiode_pin;

    tssp_detect_config_t tssp_detect_config = 
    {
        .detect_logic_level = false,
        .tssp_missed_handler = window_trigger,
        .tssp_detect_handler = pulse_detect_handler,
        .rx_en_pin = sensebe_rx_detect_config->rx_en_pin,
        .rx_in_pin = sensebe_rx_detect_config->rx_out_pin,
        .window_duration = sensebe_rx_detect_config->time_window_ms,
    };
    tssp_detect_init (&tssp_detect_config);
    
    cam_trigger_setup_t cam_trig_setup = 
    {
        .cam_trigger_done_handler = camera_unit_handler,
        .focus_pin = sensebe_rx_detect_config->focus_pin_no,
        .trigger_pin = sensebe_rx_detect_config->trigger_pin_no
    };
    cam_trigger_init (&cam_trig_setup);
}

void sensebe_rx_detect_start (void)
{
    log_printf("%s\n", __func__);
    detect_time_pass = 0;
    state = MOTION_FEEDBACK;

    device_tick_cfg tick_cfg =
    {
        MS_TIMER_TICKS_MS(SENSE_FAST_TICK_INTERVAL_MS),
        MS_TIMER_TICKS_MS(SENSE_SLOW_TICK_INTERVAL_MS),
        DEVICE_TICK_SLOW
    };
    device_tick_init(&tick_cfg);

    cam_trigger_config_t cam_trig_config = 
    {
        .setup_number = MOTION_ONLY,
        .trig_duration_100ms = 100,
        .trig_mode = CAM_TRIGGER_VIDEO,
        .trig_param1 = 1,
        .trig_param2 = 2
    };

    cam_trigger_set_trigger (&cam_trig_config);

    tssp_detect_window_detect ();
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    tssp_detect_stop ();
}

void sensebe_rx_detect_add_ticks (uint32_t interval)
{
    switch (state)
    {
        case MOTION_FEEDBACK : 
            detect_time_pass += interval;
            if(detect_time_pass >= DETECT_FEEDBACK_TIMEOUT_TICKS)
            {
                state = MOTION_IDLE;
                detect_time_pass = 0;
            }
            break;
        case MOTION_WAIT_FOR_TIMEOUT : 
            wait_window_timepassed += interval;
            if(wait_window_timepassed >= CAMERA_TIMEOUT_30S)
            {
                device_tick_switch_mode (DEVICE_TICK_SLOW);
                tssp_detect_stop ();
                tssp_detect_pulse_detect ();
                wait_window_timepassed = 0;
                state = MOTION_SYNC;
            }
            break;
        case MOTION_IDLE : 
            if(light_sense (1))
            {
                tssp_detect_window_detect ();
            }
            else
            {
                tssp_detect_stop ();
            }
            break;
        case MOTION_SYNC : 
            if(light_sense (1))
            {
                ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_SINGLE_CALL, 
                                MS_TIMER_TICKS_MS(1000), timer_1s_handler);
            }
            else
            {
                ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
            }
                
            break;
    }    
}
