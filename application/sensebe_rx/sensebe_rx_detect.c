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

enum 
{
    DISABLE,
    ENABLE,
};

/** The fast tick interval in ms in the Sense mode */
#define SENSE_FAST_TICK_INTERVAL_MS      1000
/** The slow tick interval in ms in the Sense mode */
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

#define CAMERA_TIMEOUT_30S  MS_TIMER_TICKS_MS(30000)

#define MS_TIMER_USED MS_TIMER2
#define SINGLE_SHOT_TRANSITIONS 2
#define SINGLE_SHOT_DURATION MS_TIMER_TICKS_MS(250)
#define NULL_STATE 0
#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(600000)
#define INTER_TRIG_TIME MS_TIMER_TICKS_MS(750)

static bool detect_feedback_flag = true;
static uint32_t detect_time_pass = 0;

volatile bool wait_window_flag = DISABLE;
static uint32_t wait_window_timepassed = 0;

volatile bool camera_flag = DISABLE;
void camera_unit_handler(uint32_t state)
{
    log_printf("%s\n", __func__);
    log_printf("State : %d\n", state);
    if (wait_window_flag == ENABLE)  
    {
        device_tick_process ();
    }
    if(camera_flag == ENABLE)
    {
        tssp_detect_window_detect ();
    }
}

//void system_dowm (void)
//{   
//    log_printf("%s\n",__func__);
//    tssp_detect_stop ();
//    NRF_POWER->SYSTEMOFF = (POWER_SYSTEMOFF_SYSTEMOFF_Enter 
//        << POWER_SYSTEMOFF_SYSTEMOFF_Pos) & POWER_SYSTEMOFF_SYSTEMOFF_Msk;
//}

void timer_1s (void);
void timer_200ms (void);

void timer_200ms (void)
{
    tssp_detect_stop ();
    ms_timer_start (MS_TIMER_USED, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(1000),
                    timer_1s);
}

void timer_1s (void)
{
    tssp_detect_pulse_detect ();
    ms_timer_start (MS_TIMER_USED, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(200),
                    timer_200ms);
}

void window_trigger ()
{
    log_printf("%s\n", __func__);
    {
        tssp_detect_stop ();
        if(detect_feedback_flag == true)
        {
            led_ui_single_start (LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
            wait_window_timepassed = 0;
        }
        else 
        {
            device_tick_switch_mode (DEVICE_TICK_FAST);
            log_printf("Device tick mode : %d\n", DEVICE_TICK_SAME);
            tssp_detect_pulse_detect ();
            wait_window_flag = ENABLE;
        }
        camera_flag = ENABLE;
        cam_trigger (0);
    }
}

void sync_start ()
{
    log_printf ("%s\n", __func__);
    detect_feedback_flag = true;
    detect_time_pass = 0;
    ms_timer_stop (MS_TIMER_USED);
    tssp_detect_window_detect ();
}

void pulse_detect_handler ()
{
    if(wait_window_flag == DISABLE)
    {
        sync_start ();
    }
    else
    {
        device_tick_process ();
        wait_window_timepassed = 0;
        wait_window_flag = DISABLE;   
        camera_flag = DISABLE;
    }
}

void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);

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
    
    cam_trigger_config_t cam_trig_conf = 
    {
        .cam_trigger_done_handler = camera_unit_handler,
        .no_of_setups = 1,
        .focus_pin = sensebe_rx_detect_config->focus_pin_no,
        .trigger_pin = sensebe_rx_detect_config->trigger_pin_no
    };
    cam_trigger_init (&cam_trig_conf);
}

void sensebe_rx_detect_start (void)
{
    log_printf("%s\n", __func__);
    detect_time_pass = 0;
    detect_feedback_flag = true;

    device_tick_cfg tick_cfg =
    {
        MS_TIMER_TICKS_MS(SENSE_FAST_TICK_INTERVAL_MS),
        MS_TIMER_TICKS_MS(SENSE_SLOW_TICK_INTERVAL_MS),
        DEVICE_TICK_SLOW
    };
    device_tick_init(&tick_cfg);

    cam_trigger_setup_t cam_trig_setup = 
    {
        .setup_number = 0,
        .trig_duration_100ms = 10
    };
    
    cam_trigger_t local_cam_trigger =
    {
        .trig_mode = SINGLE_SHOT,
        .trig_param1 = 0,
        .trig_param2 = 0
    };

    cam_trigger_set_trigger (&local_cam_trigger, &cam_trig_setup);

    tssp_detect_window_detect ();
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    tssp_detect_stop ();
}

void sensebe_rx_detect_add_ticks (uint32_t interval)
{
    if(detect_feedback_flag == true)
    {
        detect_time_pass += interval;
        if(detect_time_pass >= DETECT_FEEDBACK_TIMEOUT_TICKS)
        {
            detect_feedback_flag = false;
            detect_time_pass = 0;
        }
    }
    if(wait_window_flag == ENABLE)
    {
        log_printf("Here..!!\n");
        wait_window_timepassed += interval;
        if(wait_window_timepassed >= CAMERA_TIMEOUT_30S)
        {
            tssp_detect_stop ();
            tssp_detect_pulse_detect ();
            wait_window_flag = DISABLE;
            camera_flag = DISABLE;
            wait_window_timepassed = 0;
        }
    }
    
}
