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

#include "sensebe_ble.h"
#include "sensebe_tx_rx_mod.h"

#include "hal_gpio.h"
#include "ms_timer.h"
#include "log.h"
#include "led_ui.h"
#include "led_seq.h"
#include "tssp_detect.h"
#include "device_tick.h"
#include "cam_trigger.h"
#include "simple_adc.h"
#include "string.h"
#include "hal_nop_delay.h"
#include "tssp_ir_tx.h"

typedef enum
{
    MOTION_SYNC,
    MOTION_IDLE,
    MOTION_STOP,
    MOTION_MAX_STATE
}motion_detection_states_t;

typedef enum
{
    MOD_TIMER,
    MOD_MOTION,
    MOD_IR_TX,
    MAX_MODS,
}modules_t;

typedef enum 
{
    IR_SHORT,
    IR_MID,
    IR_LONG,
    IR_MAX_RANGES,
}ir_ranges_t;

typedef enum
{
    IR_SPEED0 = 5,
    IR_SPEED1 = 25,
    IR_SPEED2 = 50,
    IR_SPEED3 = 100,
}ir_speed_t;

#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(270000)

#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32

#define MAX_ADC_OUTPUT 4096

#define LIGHT_SENSE_INTERVAL_TICKS MS_TIMER_TICKS_MS(300000)

#define PULSE_REQ_FOR_SYNC 4

#define MOTION_SYNC_ON_TIME 150

#define MOTION_SYNC_OFF_TIME 850

static motion_detection_states_t motion_state = MOTION_IDLE;

static uint32_t feedback_timepassed = 0;

static sensebe_config_t sensebe_config;

static tssp_detect_config_t tssp_detect_config;

static bool arr_is_light_ok [MAX_MODS];

static bool arr_is_light_sense_req [MAX_MODS];

static bool arr_is_mod_on[MAX_MODS];

static uint32_t light_check_sense_pin = 0;

static uint32_t light_check_en_pin = 0;

static uint32_t module_tick_duration[] = {IR_SPEED0, IR_SPEED1, IR_SPEED2, IR_SPEED3};

static uint32_t timer_module_value = 0;

static led_sequences range_indicator[] =
            {LED_SEQ_RED_PULSE, LED_SEQ_ORANGE_PULSE, LED_SEQ_GREEN_PULSE,};

static ir_ranges_t tx_range = IR_SHORT;

static uint32_t ir_pwr1 = 0, ir_pwr2 =0;

/***/
void light_check (uint32_t interval);

void module_tick_handler ();

void add_ticks_feedback (uint32_t interval)
{
    feedback_timepassed += interval;
    if(feedback_timepassed >= DETECT_FEEDBACK_TIMEOUT_TICKS)
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
    }
}

void state_change_sync ()
{
    log_printf ("%s\n", __func__);
    tssp_detect_window_stop ();
}

void state_change_idle ()
{
    log_printf ("%s\n", __func__);
    tssp_detect_pulse_stop ();
    tssp_detect_window_detect ();
}

void state_change_stop ()
{
    
    log_printf ("%s\n", __func__);
    tssp_detect_window_stop ();
    tssp_detect_pulse_stop ();
}

void (* arr_state_change[]) () = {
    state_change_sync,
    state_change_idle,
    state_change_stop
};

void ir_range_short ()
{
    log_printf("%s\n",__func__);
    hal_gpio_pin_write (ir_pwr1, 0);
    hal_gpio_pin_write (ir_pwr2, 0);
}

void ir_range_mid ()
{
    log_printf("%s\n",__func__);
    hal_gpio_pin_write (ir_pwr1, 1);
    hal_gpio_pin_write (ir_pwr2, 0);
}

void ir_range_long ()
{
    log_printf("%s\n",__func__);
    hal_gpio_pin_write (ir_pwr1, 1);
    hal_gpio_pin_write (ir_pwr2, 1);
}

void (* ir_range_select[]) () ={
    ir_range_short,
    ir_range_mid,
    ir_range_long,
};

void camera_unit_handler(uint32_t trigger)
{
    log_printf("%s\n", __func__);
    switch(trigger)
    {
        case MOD_MOTION : 
            break;
        case MOD_TIMER :
            break;
    }
    
}

void timer_trigger_handler ()
{
    
    if(cam_trigger_is_on () == false)
    {
        cam_trigger (MOD_TIMER);
    }
}

void window_detect_handler ()
{
    {
        led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
        led_ui_single_start (LED_SEQ_DETECT_WINDOW, LED_UI_MID_PRIORITY, true);
        motion_state = MOTION_SYNC;
        arr_state_change[motion_state] ();
        tssp_detect_pulse_detect ();
        cam_trigger (MOD_MOTION);
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
    if(motion_state == MOTION_SYNC)
    {
        if(two_window_sync (ticks_count))
        {
            motion_state = MOTION_IDLE;
            arr_state_change[motion_state] ();
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
                led_ui_single_start (LED_SEQ_DETECT_SYNC, LED_UI_LOW_PRIORITY, true);
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
        if(arr_is_light_sense_req[MOD_MOTION])
        {
            light_sense (sensebe_config.tssp_conf.oper_time, light_intensity, MOD_MOTION);
        }
        
        //timer light check
        if(arr_is_light_sense_req[MOD_TIMER])
        {
            light_sense (sensebe_config.timer_conf.oper_time, light_intensity, MOD_TIMER);
        }
        
        if(arr_is_light_sense_req[MOD_IR_TX])
        {
            light_sense (sensebe_config.ir_tx_conf.oper_time, light_intensity, MOD_IR_TX);
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
        arr_is_light_sense_req[MOD_MOTION] = false;
        arr_is_light_ok [MOD_MOTION] = true; 
    }
    else
    {
        arr_is_light_sense_req[MOD_MOTION] = true;
    }      
    cam_trigger_config_t motion_cam_trig_config = 
    {
        .setup_number = MOD_MOTION,
        .trig_duration_100ms = sensebe_config.tssp_conf.intr_trig_timer,
        .trig_mode = sensebe_config.tssp_conf.mode,
        .trig_param1 = sensebe_config.tssp_conf.larger_value,
        .trig_param2 = sensebe_config.tssp_conf.smaller_value,
    };
    cam_trigger_set_trigger (&motion_cam_trig_config);

    tssp_detect_config.window_duration_ticks =
        sensebe_config.tssp_conf.detect_window;
    tssp_detect_init (&tssp_detect_config);

    motion_state = MOTION_SYNC;
    arr_is_mod_on[MOD_MOTION] = true;
}

void motion_module_add_ticks ()
{
    log_printf("Machine State : %d\n", motion_state);
    if(arr_is_light_ok [MOD_MOTION] == false)
    {
        motion_state = MOTION_STOP;
    }
    else if(motion_state == MOTION_STOP)
    {
        motion_state = MOTION_SYNC;
    }
    arr_state_change[motion_state]();
}

void motion_module_add_mod_ticks ()
{
    static bool is_rx_on = true;
    static uint32_t mod_ticks;
    mod_ticks += module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed];
    if(motion_state == MOTION_SYNC)
    {
        if(is_rx_on == true && mod_ticks >= MOTION_SYNC_ON_TIME)
        {
            tssp_detect_pulse_stop ();
            is_rx_on = false;
            mod_ticks = 0;
        }
        else if(is_rx_on == false && mod_ticks >= MOTION_SYNC_OFF_TIME)
        {
            tssp_detect_pulse_detect ();
            is_rx_on = true;
            mod_ticks = 0;
        }
    }
    else if(motion_state == MOTION_IDLE)
    {
        mod_ticks = 0;
    }
}

void motion_module_stop ()
{
    motion_state = MOTION_STOP;
    arr_state_change[motion_state] ();

    arr_is_mod_on [MOD_MOTION] = false;
    arr_is_light_sense_req[MOD_MOTION] = false;
    arr_is_light_ok[MOD_MOTION] = false;
}

void timer_module_start ()
{
    oper_time_t timer_oper_time = sensebe_config.timer_conf.oper_time;

    if((timer_oper_time.day_or_night == 1 && timer_oper_time.threshold == 0b0000000)||
    (timer_oper_time.day_or_night == 0 && timer_oper_time.threshold == 0b1111111))
    {
        arr_is_light_sense_req[MOD_TIMER] = false;
        arr_is_light_ok [MOD_TIMER] = true; 
    }
    else
    {
        arr_is_light_sense_req[MOD_TIMER] = true;
    }      
    cam_trigger_config_t timer_cam_trig_config = 
    {
        .setup_number = MOD_TIMER,
        .trig_duration_100ms = 0,
        .trig_mode = sensebe_config.timer_conf.mode,
        .trig_param1 = sensebe_config.timer_conf.larger_value,
        .trig_param2 = sensebe_config.timer_conf.smaller_value
    };
    cam_trigger_set_trigger (&timer_cam_trig_config);
    
    timer_module_value = sensebe_config.timer_conf.timer_interval * 100;
    
    arr_is_mod_on[MOD_TIMER] = true;
    
}

void timer_module_add_ticks ()
{
    if(arr_is_light_ok [MOD_TIMER] == true)
    {
        arr_is_mod_on[MOD_TIMER] = true;
    }
    else 
    {
        arr_is_mod_on[MOD_TIMER] = false;
    }
}

void timer_module_add_mod_ticks ()
{
    static uint32_t mod_ticks;
    mod_ticks += module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed];
    if(mod_ticks > timer_module_value)
    {
        timer_trigger_handler ();
    }
}

void timer_module_stop ()
{
    arr_is_mod_on [MOD_TIMER] = false;
    arr_is_light_sense_req[MOD_TIMER] = false;
    arr_is_light_ok[MOD_TIMER] = false;
}

void ir_tx_module_start ()
{
    oper_time_t ir_tx_oper_time = sensebe_config.ir_tx_conf.oper_time;

    if((ir_tx_oper_time.day_or_night == 1 && ir_tx_oper_time.threshold == 0b0000000)||
    (ir_tx_oper_time.day_or_night == 0 && ir_tx_oper_time.threshold == 0b1111111))
    {
        arr_is_light_sense_req[MOD_IR_TX] = false;
        arr_is_light_ok [MOD_IR_TX] = true; 
    }
    else
    {
        arr_is_light_sense_req[MOD_IR_TX] = true;
    }      
    
    if(sensebe_config.ir_tx_conf.ir_tx_pwr == IR_MAX_RANGES)
    {
        sensebe_config.ir_tx_conf.ir_tx_pwr = IR_LONG;
    }

    tx_range = sensebe_config.ir_tx_conf.ir_tx_pwr;
    ir_range_select[tx_range]();    
    led_ui_single_start (range_indicator[tx_range], LED_UI_LOW_PRIORITY, true);

    arr_is_mod_on[MOD_IR_TX] = true;
}

void ir_tx_module_add_ticks ()
{
    if(arr_is_light_ok[MOD_IR_TX] == true)
    {
        arr_is_mod_on[MOD_IR_TX] = true;
    }
    else
    {
        arr_is_mod_on[MOD_IR_TX] = false;
    }
}

void ir_tx_module_add_mod_ticks ()
{
    tssp_ir_tx_start ();
}

void ir_tx_module_stop ()
{
    tssp_ir_tx_stop ();
    arr_is_mod_on[MOD_IR_TX] = false;
    arr_is_light_sense_req[MOD_IR_TX] = false;
    arr_is_light_ok[MOD_IR_TX] = false;
}

void module_tick_handler ()
{
    if(arr_is_mod_on[MOD_TIMER] == true)
    {
        timer_module_add_mod_ticks ();
    }
    if(arr_is_mod_on[MOD_MOTION] == true)
    {
        motion_module_add_mod_ticks ();
    }
    if(arr_is_mod_on[MOD_IR_TX] == true)
    {
        ir_tx_module_add_mod_ticks ();
    }
}

void sensebe_tx_rx_init (sensebe_tx_rx_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);
    
    //Assign Enable and sense pins
    light_check_sense_pin = sensebe_rx_detect_config->rx_detect_config.photodiode_pin;
    hal_gpio_cfg_input (light_check_sense_pin, HAL_GPIO_PULL_DOWN);
    light_check_en_pin = sensebe_rx_detect_config->rx_detect_config.photodiode_en_pin;
    hal_gpio_cfg_output (light_check_en_pin, 0);
    
    memcpy (&sensebe_config, sensebe_rx_detect_config->sensebe_config,
            sizeof(sensebe_config_t));
    
    tssp_detect_config_t local_tssp_detect_config = 
    {
        .detect_logic_level = false,
        .tssp_missed_handler = window_detect_handler,
        .tssp_detect_handler = pulse_detect_handler,
        .rx_en_pin = sensebe_rx_detect_config->rx_detect_config.rx_en_pin,
        .rx_in_pin = sensebe_rx_detect_config->rx_detect_config.rx_out_pin,
        .window_duration_ticks = 
            (sensebe_config.tssp_conf.detect_window ),
    };
    memcpy (&tssp_detect_config, &local_tssp_detect_config,
            sizeof(tssp_detect_config_t));
    
    cam_trigger_setup_t cam_trig_setup = 
    {
        .cam_trigger_done_handler = camera_unit_handler,
        .focus_pin = sensebe_rx_detect_config->rx_detect_config.focus_pin_no,
        .trigger_pin = sensebe_rx_detect_config->rx_detect_config.trigger_pin_no
    };
    cam_trigger_init (&cam_trig_setup);    
    
    tssp_ir_tx_init (sensebe_rx_detect_config->tx_transmit_config.tx_en_pin,
                     sensebe_rx_detect_config->tx_transmit_config.tx_in_pin);
    
    ir_pwr1 = sensebe_rx_detect_config->tx_transmit_config.tx_pwr1;
    ir_pwr2 = sensebe_rx_detect_config->tx_transmit_config.tx_pwr2;
    

    hal_gpio_cfg_output (ir_pwr1, 0);
    hal_gpio_cfg_output (ir_pwr2, 0);
        
}

void sensebe_tx_rx_start (void)
{
    log_printf("%s\n", __func__);
    feedback_timepassed = 0;
    
    //Check if light sense is required
    
    log_printf(" Trig Config : %d\n ", sensebe_config.trig_conf);
    
    if(sensebe_config.trig_conf != MOTION_ONLY)
    {
        timer_module_start ();
    }
    else
    {
        timer_module_stop ();
    }
    
    if(sensebe_config.trig_conf != TIMER_ONLY)
    {
        motion_module_start ();
    }
    else
    {
        motion_module_stop ();
    }
    
    if(sensebe_config.ir_tx_conf.is_enable == 1)
    {
        ir_tx_module_start ();
    }
    else
    {
        ir_tx_module_stop ();
    }
    
    ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_REPEATED_CALL,
        MS_TIMER_TICKS_MS(module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed])
        , module_tick_handler);
    
    light_check (LIGHT_SENSE_INTERVAL_TICKS);
}

void sensebe_tx_rx_stop (void)
{
    log_printf("%s\n", __func__);
    cam_trigger_stop ();
    motion_module_stop ();
    timer_module_stop ();
    ir_tx_module_stop ();
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
}

void sensebe_tx_rx_add_ticks (uint32_t interval)
{
    add_ticks_feedback (interval);

    if(arr_is_light_sense_req [MOD_MOTION] == true ||
       arr_is_light_sense_req [MOD_TIMER] == true ||
       arr_is_light_sense_req [MOD_IR_TX])
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
    if(sensebe_config.ir_tx_conf.is_enable == 1)
    {
        ir_tx_module_add_ticks ();
    }
}

void sensebe_tx_rx_update_config (sensebe_config_t * update_sensebe_config)
{
    memcpy (&sensebe_config, update_sensebe_config, sizeof(sensebe_config_t));
}

sensebe_config_t * sensebe_tx_rx_last_config ()
{
    return &sensebe_config;
}

void sensebe_tx_rx_swicht_range ()
{
    tx_range = (tx_range + 1)%IR_MAX_RANGES;
    led_ui_single_start (range_indicator[tx_range], LED_UI_LOW_PRIORITY, true);

    ir_range_select[tx_range]();
    sensebe_config.ir_tx_conf.ir_tx_pwr = tx_range;
}
