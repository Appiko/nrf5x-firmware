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
#include "sensebe_ble.h"
#include "sensebe_rx_detect.h"
#include "log.h"
#include "led_ui.h"
#include "led_seq.h"
#include "tssp_detect.h"
#include "device_tick.h"
#include "cam_trigger.h"
#include "simple_adc.h"
#include "string.h"
#include "hal_nop_delay.h"

typedef enum
{
    MOTION_SYNC,
    MOTION_IDLE,
    MOTION_MAX_STATE
}motion_detection_states_t;

typedef enum {
    
    ONE_SHOT,
    THREE_SHOT,
    FIVE_SHOT,
    MAX_SHOTS
}shot_count_t;

/** The fast tick interval in ms in the Sense mode */
#define SENSE_FAST_TICK_INTERVAL_MS      1
/** The slow tick interval in ms in the Sense mode */
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

#define CAMERA_TIMEOUT_30S  MS_TIMER_TICKS_MS(30000)

#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(600000)

#define PULSE_REQ_FOR_SYNC 3

volatile motion_detection_states_t state = MOTION_IDLE;

static shot_count_t shot_count = ONE_SHOT;

static led_sequences arr_button_seq[] = {LED_SEQ_RED_PULSE, 
        LED_SEQ_ORANGE_PULSE, LED_SEQ_GREEN_PULSE};

static uint32_t feedback_timepassed = 0;

static uint32_t wait_window_timepassed = 0;

void timer_1s_handler (void);
void timer_200ms_handler (void);

void state_control_motion_sync (uint32_t interval)
{
    log_printf ("%d : %s\n", state,__func__);

    tssp_detect_window_stop ();

    ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_SINGLE_CALL, 
                    MS_TIMER_TICKS_MS(300), timer_1s_handler);

}

void state_control_motion_idle (uint32_t interval)
{

    log_printf ("%d : %s\n", state,__func__);

    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
    tssp_detect_window_detect ();

}

void (* arr_state_control_motion[]) (uint32_t interval) = {
    state_control_motion_sync,
    state_control_motion_idle,
};


void timer_200ms_handler (void)
{
    tssp_detect_pulse_stop ();
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
    {
    }
    
}

void window_detect_handler ()
{
//    log_printf ("%s\n", __func__);
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        led_ui_single_start (LED_SEQ_DETECT_WINDOW, LED_UI_MID_PRIORITY, true);
        state = MOTION_SYNC;
        arr_state_control_motion[state] (0);
        cam_trigger (shot_count);
    }
}

bool compare_percent(uint32_t data, uint32_t ref, float per)
{
    if((ref-(ref*per/100))<=data && data<=(ref+(ref*per/100)))
    {
        return true;
    }
    else
    {
        return false;    
    }
}

bool two_window_sync (uint32_t ticks)
{
    static uint32_t previous_pulse_tick = 0, current_pulse_tick = 0;
    static uint32_t pulse_diff_window[] = {0,0}, pulse_diff_window_cnt = 0;
    static uint32_t pulse_cnt = PULSE_REQ_FOR_SYNC;
    if(pulse_cnt == PULSE_REQ_FOR_SYNC)
    {
        previous_pulse_tick = ticks;
        pulse_cnt --;
    }
    else if(pulse_cnt > 0)
    {
        current_pulse_tick = ticks;
        pulse_diff_window[pulse_diff_window_cnt] = ((current_pulse_tick + (1<<24))- previous_pulse_tick)
             & 0x00FFFFFF;
        previous_pulse_tick = current_pulse_tick;
        pulse_diff_window_cnt++;
        pulse_cnt--;
    }
    else if(pulse_cnt == 0)
    {
        pulse_diff_window_cnt = 0;
        log_printf("Window[0]: %d\n", pulse_diff_window[0]);
        log_printf("Window[1]: %d\n", pulse_diff_window[1]);
        pulse_cnt = PULSE_REQ_FOR_SYNC;
        return compare_percent (pulse_diff_window[1], pulse_diff_window[0], 10);
    }
    
    return false;
}

void pulse_detect_handler (uint32_t ticks_count)
{
//    log_printf("%s\n", __func__);
    if(state == MOTION_SYNC)
    {
        if(two_window_sync (ticks_count))
        {
            state = MOTION_IDLE;
            arr_state_control_motion[state] (0);
//            return;
        }
        else
        {
            tssp_detect_pulse_detect ();
            led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        }
    }
    else if(state == MOTION_IDLE)
    {
        if(feedback_timepassed < DETECT_FEEDBACK_TIMEOUT_TICKS)
        {
            led_ui_loop_start (LED_SEQ_DETECT_PULSE, LED_UI_LOW_PRIORITY);
        }
    }
}

void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);
    
    tssp_detect_config_t tssp_detect_config = 
    {
        .detect_logic_level = false,
        .tssp_missed_handler = window_detect_handler,
        .tssp_detect_handler = pulse_detect_handler,
        .rx_en_pin = sensebe_rx_detect_config->rx_en_pin,
        .rx_in_pin = sensebe_rx_detect_config->rx_out_pin,
        .window_duration_ticks = 5,
    };
    tssp_detect_init (&tssp_detect_config);
    
    cam_trigger_setup_t cam_trig_setup = 
    {
        .cam_trigger_done_handler = camera_unit_handler,
        .focus_pin = sensebe_rx_detect_config->focus_pin_no,
        .trigger_pin = sensebe_rx_detect_config->trigger_pin_no
    };
    cam_trigger_init (&cam_trig_setup);  
    
    cam_trigger_config_t one_shot_config = 
    {
        .setup_number = ONE_SHOT,
        .trig_duration_100ms = 30,
        .trig_mode = CAM_TRIGGER_LONG_PRESS,
        .trig_param1 = 30,
        .trig_param2 = 0,
    };
    cam_trigger_set_trigger (&one_shot_config);

    cam_trigger_config_t three_shot_config = 
    {
        .setup_number = THREE_SHOT,
        .trig_duration_100ms = 20,
        .trig_mode = CAM_TRIGGER_MULTI_SHOT,
        .trig_param1 = 5,
        .trig_param2 = 3,
    };
    cam_trigger_set_trigger (&three_shot_config);

    cam_trigger_config_t five_shot_config = 
    {
        .setup_number = FIVE_SHOT,
        .trig_duration_100ms = 30,
        .trig_mode = CAM_TRIGGER_MULTI_SHOT,
        .trig_param1 = 5,
        .trig_param2 = 5,
    };
    cam_trigger_set_trigger (&five_shot_config);
}

void sensebe_rx_detect_start (void)
{
    log_printf("%s\n", __func__);
    led_ui_single_start (arr_button_seq[shot_count], LED_UI_HIGH_PRIORITY, true);
    feedback_timepassed = 0;
    wait_window_timepassed = 0;
    state = MOTION_SYNC;
    arr_state_control_motion[state] (0);
    
    
    device_tick_cfg tick_cfg =
    {
        MS_TIMER_TICKS_MS(SENSE_FAST_TICK_INTERVAL_MS),
        MS_TIMER_TICKS_MS(SENSE_SLOW_TICK_INTERVAL_MS),
        DEVICE_TICK_SLOW
    };
    device_tick_init(&tick_cfg);
    
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    cam_trigger_stop ();
    tssp_detect_pulse_stop ();
    tssp_detect_window_stop ();
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
}

void sensebe_rx_detect_add_ticks (uint32_t interval)
{
    log_printf("Machine State : %d\n", state);
    feedback_timepassed += interval;
    if(feedback_timepassed >= DETECT_FEEDBACK_TIMEOUT_TICKS)
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        feedback_timepassed = 0;
    }
    
//    arr_state_control_motion[state](interval);
}

void sensebe_rx_detect_next_setup (void)
{
    shot_count = ((uint32_t)shot_count + 1)%MAX_SHOTS;
    led_ui_single_start (arr_button_seq[shot_count], LED_UI_HIGH_PRIORITY, true);
}

