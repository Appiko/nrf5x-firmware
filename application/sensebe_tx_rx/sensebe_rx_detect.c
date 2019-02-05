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
    MOTION_STOP,
    MOTION_MAX_STATE
}motion_detection_states_t;

/** The fast tick interval in ms in the Sense mode */
#define SENSE_FAST_TICK_INTERVAL_MS      1000
/** The slow tick interval in ms in the Sense mode */
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(270000)

#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32

#define MAX_ADC_OUTPUT 4096

#define LIGHT_SENSE_INTERVAL_TICKS MS_TIMER_TICKS_MS(300000)

#define PULSE_REQ_FOR_SYNC 4

static motion_detection_states_t state = MOTION_IDLE;

static uint32_t feedback_timepassed = 0;

static sensebe_config_t sensebe_config;

static tssp_detect_config_t tssp_detect_config;

static bool arr_is_light_ok [MOTION_AND_TIMER];

static bool arr_is_light_sense_req [MOTION_AND_TIMER];

static uint32_t light_check_sense_pin = 0;

static uint32_t light_check_en_pin = 0;

/***/
void light_check (uint32_t interval);

void timer_1s_handler (void);
void timer_200ms_handler (void);

void add_ticks_feedback (uint32_t interval)
{
    log_printf ("%s\n", __func__);
    feedback_timepassed += interval;
    if(feedback_timepassed >= DETECT_FEEDBACK_TIMEOUT_TICKS)
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
    }
}

void state_change_sync ()
{
    log_printf ("%s\n", __func__);
    {
        {
            tssp_detect_window_stop ();
            ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_SINGLE_CALL, 
                            MS_TIMER_TICKS_MS(1000), timer_1s_handler);
        }
    }

}

void state_change_idle ()
{
    log_printf ("%s\n", __func__);
    tssp_detect_pulse_stop ();
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
    tssp_detect_window_detect ();
}

void state_change_stop ()
{
    
    log_printf ("%s\n", __func__);
    tssp_detect_window_stop ();
    tssp_detect_pulse_stop ();
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
}

void (* arr_state_change[]) () = {
    state_change_sync,
    state_change_idle,
    state_change_stop
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
    switch(trigger)
    {
        case MOTION_ONLY : 
            break;
        case TIMER_ONLY :
            break;
    }
    
}

void timer_trigger_handler ()
{
    
    if(cam_trigger_is_on () == false)
    {
        cam_trigger (TIMER_ONLY);
    }
}
void window_detect_handler ()
{
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        led_ui_single_start (LED_SEQ_DETECT_WINDOW, LED_UI_HIGH_PRIORITY, true);
        state = MOTION_SYNC;
        arr_state_change[state] ();
        tssp_detect_pulse_detect ();
        cam_trigger (MOTION_ONLY);
    }
}

bool compare_percent(uint32_t data, uint32_t ref, uint32_t margin)
{
    if((ref-margin)<=data && data<=(ref+margin))
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
    static uint32_t pulse_diff_window[] = {0,0,0}, pulse_diff_window_cnt = 0;
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
        log_printf("Window[2]: %d\n", pulse_diff_window[2]);
        pulse_cnt = PULSE_REQ_FOR_SYNC;
        return (compare_percent (pulse_diff_window[1], pulse_diff_window[0], 2)
            && compare_percent (pulse_diff_window[2], pulse_diff_window[1], 2));
    }
    
    return false;
}

void pulse_detect_handler (uint32_t ticks_count)
{
    if(state == MOTION_SYNC)
    {
        if(two_window_sync (ticks_count))
        {
            state = MOTION_IDLE;
            arr_state_change[state] ();
            if(feedback_timepassed < DETECT_FEEDBACK_TIMEOUT_TICKS)
            {
                led_ui_loop_start (LED_SEQ_DETECT_PULSE, LED_UI_LOW_PRIORITY);
            }
        }
        else
        {
            tssp_detect_pulse_detect ();
            if(feedback_timepassed < DETECT_FEEDBACK_TIMEOUT_TICKS)
            {
                led_ui_single_start (LED_SEQ_GREEN_PULSE, LED_UI_MID_PRIORITY, true);
            }
            led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        }
    }
}

void light_sense (oper_time_t oper_time, uint32_t light_intensity, uint32_t trigger)
{
    uint8_t light_sense_config = oper_time.day_or_night;
    uint32_t light_threshold =
            (uint32_t)((oper_time.threshold) * LIGHT_THRESHOLD_MULTIPLY_FACTOR);

    //respective light check
        //Day and its brighter than the threshold
    if(((light_sense_config == 1) && (light_intensity >= light_threshold))
            ||  //Night and its dimmer than the threshold
       ((light_sense_config == 0) && (light_intensity <= light_threshold)))
    //assgin to respective light flag
    {
        arr_is_light_ok[trigger] = true;
    }
    else
    {
        arr_is_light_ok[trigger] = false;
    }
}

void light_check (uint32_t interval)
{
    static uint32_t timepassed = 0;
    timepassed += interval;
    if(timepassed >= LIGHT_SENSE_INTERVAL_TICKS)
    {
        static uint32_t light_intensity;
        //Enable light sense module
        hal_gpio_pin_set (light_check_en_pin);
        //Take light reading
        hal_nop_delay_ms (3);
        light_intensity = (MAX_ADC_OUTPUT - simple_adc_get_value (SIMPLE_ADC_GAIN1_6,
                                                light_check_sense_pin));
        //motion light check
        if(arr_is_light_sense_req[MOTION_ONLY])
        {
            light_sense (sensebe_config.tssp_conf.oper_time, light_intensity, MOTION_ONLY);
        }
        
        //timer light check
        if(arr_is_light_sense_req[TIMER_ONLY])
        {
            light_sense (sensebe_config.timer_conf.oper_time, light_intensity, TIMER_ONLY);
        }
        
        //Disable light sense module
        hal_gpio_pin_clear (light_check_en_pin);
        timepassed = 0;
    }
}

void motion_module_start ()
{
    oper_time_t motion_oper_time = sensebe_config.tssp_conf.oper_time;

    if((motion_oper_time.day_or_night == 1 && motion_oper_time.threshold == 0b0000000)||
    (motion_oper_time.day_or_night == 0 && motion_oper_time.threshold == 0b1111111))
    {
        arr_is_light_sense_req[MOTION_ONLY] = false;
        arr_is_light_ok [MOTION_ONLY] = true; 
    }
    else
    {
        arr_is_light_sense_req[MOTION_ONLY] = true;
    }      
    cam_trigger_config_t motion_cam_trig_config = 
    {
        .setup_number = MOTION_ONLY,
        .trig_duration_100ms = sensebe_config.tssp_conf.intr_trig_timer,
        .trig_mode = sensebe_config.tssp_conf.mode,
        .trig_param1 = sensebe_config.tssp_conf.larger_value,
        .trig_param2 = sensebe_config.tssp_conf.smaller_value,
    };
    cam_trigger_set_trigger (&motion_cam_trig_config);

    tssp_detect_config.window_duration_ticks =
        sensebe_config.tssp_conf.detect_window;
    tssp_detect_init (&tssp_detect_config);

    tssp_detect_window_detect ();        
}

void motion_module_add_ticks ()
{
    log_printf("Machine State : %d\n", state);
    if(arr_is_light_ok [MOTION_ONLY] == false)
    {
        state = MOTION_STOP;
    }
    else if(state == MOTION_STOP)
    {
        state = MOTION_SYNC;
    }
    arr_state_change[state]();
}

void timer_module_start ()
{
    oper_time_t timer_oper_time = sensebe_config.timer_conf.oper_time;

    if((timer_oper_time.day_or_night == 1 && timer_oper_time.threshold == 0b0000000)||
    (timer_oper_time.day_or_night == 0 && timer_oper_time.threshold == 0b1111111))
    {
        arr_is_light_sense_req[TIMER_ONLY] = false;
        arr_is_light_ok [TIMER_ONLY] = true; 
    }
    else
    {
        arr_is_light_sense_req[TIMER_ONLY] = true;
    }      
    cam_trigger_config_t timer_cam_trig_config = 
    {
        .setup_number = TIMER_ONLY,
        .trig_duration_100ms = 0,
        .trig_mode = sensebe_config.timer_conf.mode,
        .trig_param1 = sensebe_config.timer_conf.larger_value,
        .trig_param2 = sensebe_config.timer_conf.smaller_value
    };
    cam_trigger_set_trigger (&timer_cam_trig_config);

    ms_timer_start (SENSEBE_TIMER_MODE_MS_TIMER, MS_REPEATED_CALL,
                    MS_TIMER_TICKS_MS(sensebe_config.timer_conf.timer_interval * 100),
                    timer_trigger_handler);
}

void timer_module_add_ticks ()
{
    if(arr_is_light_ok [TIMER_ONLY] == true)
    {
        if(ms_timer_get_on_status (SENSEBE_TIMER_MODE_MS_TIMER) == false)
        {
            ms_timer_start (SENSEBE_TIMER_MODE_MS_TIMER, MS_REPEATED_CALL, 
                            MS_TIMER_TICKS_MS(sensebe_config.timer_conf.timer_interval * 100),
                            timer_trigger_handler);
        }
    }
    else 
    {
        ms_timer_stop (SENSEBE_TIMER_MODE_MS_TIMER);
    }
}
void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);
    
    //Assign Enable and sense pins
    light_check_sense_pin = sensebe_rx_detect_config->photodiode_pin;
    hal_gpio_cfg_input (light_check_sense_pin, HAL_GPIO_PULL_DOWN);
    light_check_en_pin = sensebe_rx_detect_config->photodiode_en_pin;
    hal_gpio_cfg_output (light_check_en_pin, 0);
    
    memcpy (&sensebe_config, sensebe_rx_detect_config->init_sensebe_config,
            sizeof(sensebe_config_t));
    
    tssp_detect_config_t local_tssp_detect_config = 
    {
        .detect_logic_level = false,
        .tssp_missed_handler = window_detect_handler,
        .tssp_detect_handler = pulse_detect_handler,
        .rx_en_pin = sensebe_rx_detect_config->rx_en_pin,
        .rx_in_pin = sensebe_rx_detect_config->rx_out_pin,
        .window_duration_ticks = 
            MS_TIMER_TICKS_MS(sensebe_config.tssp_conf.detect_window ),
    };
    memcpy (&tssp_detect_config, &local_tssp_detect_config,
            sizeof(tssp_detect_config_t));
    
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
    feedback_timepassed = 0;
    state = MOTION_SYNC;
    arr_state_change[state] ();
    
    //Check if light sense is required
    
    log_printf(" Trig Config : %d\n ", sensebe_config.trig_conf);
    
    if(sensebe_config.trig_conf != MOTION_ONLY)
    {
        timer_module_start ();
    }
    else
    {
        arr_is_light_sense_req[TIMER_ONLY] = false;
    }
    
    if(sensebe_config.trig_conf != TIMER_ONLY)
    {
        motion_module_start ();
    }
    else
    {
        arr_is_light_sense_req[MOTION_ONLY] = false;
    }
    
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    cam_trigger_stop ();
    tssp_detect_pulse_stop ();
    tssp_detect_window_stop ();
    ms_timer_stop (SENSEBE_TIMER_MODE_MS_TIMER);
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
}

void sensebe_rx_detect_add_ticks (uint32_t interval)
{
    add_ticks_feedback (interval);

    if(arr_is_light_sense_req[MOTION_ONLY] == true ||
       arr_is_light_sense_req [TIMER_ONLY] == true)
    {
        light_check (interval);
    }
    
    if(sensebe_config.trig_conf != TIMER_ONLY)
    {
        motion_module_add_ticks ();
    }
    if(sensebe_config.trig_conf != MOTION_ONLY)
    {
        timer_module_add_ticks (); 
    }
}

void sensebe_rx_detect_update_config (sensebe_config_t * update_sensebe_config)
{
    memcpy (&sensebe_config, update_sensebe_config, sizeof(sensebe_config_t));
}

sensebe_config_t * sensebe_rx_detect_last_config ()
{
    return &sensebe_config;
}

