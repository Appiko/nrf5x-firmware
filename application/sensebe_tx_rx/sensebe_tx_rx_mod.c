/* 
 * File:   sensebe_tx_rx_mod.c
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
#include "sensebe_store_config.h"

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

/***********MACROS***********/
/** Time upto which LED feedback is to be given in motion detection mode */
#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(270000)
/** Multiplying factor for light intensity value received from BLE structures */
#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32
/** Max ADC numeric output value possible */
#define MAX_ADC_OUTPUT 4096
/** Cycle time after which light conditions has to be checked */
#define LIGHT_SENSE_INTERVAL_TICKS MS_TIMER_TICKS_MS(300000)
/** Number of pulses required for sync while module is in motion sync mode.  */
#define PULSE_REQ_FOR_SYNC 4
/** On time for TSSP receiver while module is in motion sync mode */
#define MOTION_SYNC_ON_TIME 150
/** Off time for TSSP receiver while module is in motion sync mode */
#define MOTION_SYNC_OFF_TIME 850

/***********ENUMS***********/
/** List of states for Motion Detection module. */
typedef enum
{
    /**Synchronization state*/
    MOTION_SYNC,
    /**Idle State*/
    MOTION_IDLE,
    /**Stop State*/
    MOTION_STOP,
    /**Maximum States Limit*/
    MAX_MOTION_STATE
}motion_detection_states_t;

/** List of all submodules which are being managed by this module */
typedef enum
{
    /**Timer Module*/
    MOD_TIMER,
    /**Motion Detection Module*/
    MOD_MOTION,
    /**IR transmission module*/
    MOD_IR_TX,
    /**Maximum number of modules*/
    MAX_MODS,
}modules_t;

/** List of distance settings available for IR transmitter module. */
typedef enum 
{
    /**Short Distance transmission*/
    IR_SHORT,
    /**Mid Distance transmission */
    IR_MID,
    /**Long Distance transmission*/
    IR_LONG,
    /**Maximum number of possible distances*/
    MAX_IR_RANGES,
}ir_ranges_t;

/** List of all possible time delays between two wake-up events. */
typedef enum
{
    /**Wake up time 5ms*/
    MOD_FREQ0 = TSSP_DETECT_TICKS_MS(5),
    /**Wake up time 25ms*/
    MOD_FREQ1 = TSSP_DETECT_TICKS_MS(25),
    /**Wake up time 50ms*/
    MOD_FREQ2 = TSSP_DETECT_TICKS_MS(50),
    /**Wake up time 100ms*/
    MOD_FREQ3 = TSSP_DETECT_TICKS_MS(100),
    /**Maximum number of frequencies possible*/
    MAX_MOD_FREQ = 4,
}module_freq_t;

/***********VARIABLE***********/
/**Global variable used to check if LED feedback is required or not*/
static uint32_t feedback_timepassed = 0;
/**Global sensebe_config_t which is to be used in this module*/
static sensebe_config_t sensebe_config;
/**Array of the wake up times*/
static uint32_t arr_module_tick_duration[] = {MOD_FREQ0, MOD_FREQ1, MOD_FREQ2, MOD_FREQ3};
/**Array of light status flags*/
static bool arr_is_light_ok [MAX_MODS];
/**Array of flags to keep track if light check is required or not*/
static bool arr_is_light_sense_req [MAX_MODS];
/**Array of module status flag*/
static bool arr_is_mod_on[MAX_MODS];
/**Global TSSP configuration which is to be modified and reused.*/
static tssp_detect_config_t tssp_detect_config;
/**Global variable to store synchronization time for TSSP detect module*/
static uint32_t tssp_detect_sync_time;
/**Global variable used to keep track of motion detection module's state*/
static motion_detection_states_t motion_state = MOTION_IDLE;
/**Global variable used to store value after which timer trigger should be generated*/
static uint32_t timer_module_value = 0;
/**Array of LED patterns related to IR transmission range */
static led_sequences arr_range_indicator[] =
            {LED_SEQ_RED_PULSE, LED_SEQ_ORANGE_PULSE, LED_SEQ_GREEN_PULSE,};
/**Global variable to keep track of IR transmission range*/
static ir_ranges_t tx_range = IR_SHORT;
/**Global variables to store pin numbers of IR transmission LED control pins*/
static uint32_t ir_pwr1 = 0, ir_pwr2 =0;
/**Global variable to store Analog pin number of light sense pin*/
static uint32_t light_check_sense_pin = 0;
/**Global variable to store pin number of Light sensor control pin*/
static uint32_t light_check_en_pin = 0;

/***********FUNCTIONS***********/
/** Motion Detection Module Related Functions. */
/**Function to start motion detection module*/
void motion_module_start (void);
/**Function to handle add_ticks event for motion detection module*/
void motion_module_add_ticks (void);
/**Function to handle add_mod_ticks event for motion detection module*/
void motion_module_add_mod_ticks (void);
/**Function to stop motion detection module*/
void motion_module_stop (void);

/**Function used to put motion detection module in Sync state*/
void state_change_sync (void);
/**Function used to put motion detection module in Idle state*/
void state_change_idle (void);
/**Function used to put motion detection module in Stop state*/
void state_change_stop (void);
/**Array of function pointers used to change state*/
void (* arr_state_change[]) () = {
    state_change_sync,
    state_change_idle,
    state_change_stop
};

/**Handler which will get provoked when window is detected*/
void window_detect_handler ();
/**Handler which will get provoked when pulse is detected*/
void pulse_detect_handler (uint32_t ticks_count);


/** Timer Module Related Functions. */
/**Function to start timer module*/
void timer_module_start (void);
/**Function to handle add_ticks event for timer module*/
void timer_module_add_ticks (void);
/**Function to handle add_mod_ticks event for timer module*/
void timer_module_add_mod_ticks (void);
/**Function to stop timer module*/
void timer_module_stop (void);

/**Function which is to be called when timer trigger should happen*/
void timer_trigger_handler ();

/** IR Transmitter Module Related Functions */
/**Function to start IR transmitter module*/
void ir_tx_module_start (void);
/**Function to handle add_ticks event for IR transmitter module*/
void ir_tx_module_add_ticks (void);
/**Function to handle add_mod_ticks event for IR transmitter module*/
void ir_tx_module_add_mod_ticks (void);
/**Function to stop IR transmitter module*/
void ir_tx_module_stop (void);

/**Function to change IR transmission range to short range*/
void ir_range_short (void);
/**Function to change IR transmission range to mid range*/
void ir_range_mid (void);
/**Function to change IR transmission range to long range*/
void ir_range_long (void);
/**Array of function pointers used to change range of IR transmission*/
void (* arr_ir_range_select[]) () ={
    ir_range_short,
    ir_range_mid,
    ir_range_long,
};

/** Light Sensing Related Functions. */
/***/
/**
 * @brief Function to check current Light intensity
 * @param oper_time Light conditions given by BLE config
 * @param light_intensity current Light intensity
 * @param module Module for which this light testing is being done.
 */
void light_check (oper_time_t oper_time, uint32_t light_intensity, uint32_t module);
/**
 * @brief Function to handle add_ticks_event for Light sensing module
 * @param interval Number of MS_TIMER_TICKS happened since last event
 */
void light_sense_add_ticks (uint32_t interval);

/** Support Functions */
/**
 * @brief Callbcak function which will be called when cam_trigger module finishes it's operation.
 * @param trigger Module which triggered this cam_trigger operation
 */
void camera_unit_handler(uint32_t trigger);
/**
 * @brief Function to compare two values with some margin
 * @param data Data which is to be compared
 * @param ref Reference with which data is to be compared
 * @param margin Value by which data can vary from reference without failing the test
 * @return comparison result
 * @retval true Data is within the margins from reference
 * @retval false Data is not within the margins from reference
 */
bool compare_margin(uint32_t data, uint32_t ref, uint32_t margin);
/**
 * @brief Function to see if four pulses approximately have same time difference   
 * @param ticks Ticks since last pulse.
 * @return Result of pulse duration comparison.
 * @retval true Four pulses have approximately equal delay.
 * @retval false Fout pulses do not have approximately equal delay. 
 */
bool three_window_sync (uint32_t ticks);
/**
 * @brief Function to handle add_tick event for LED feedback functionality.
 * @param interval MS_TIMER_TICKS since last add_ticks event
 */
void add_ticks_feedback (uint32_t interval);

/** Module Clock Handler */
/**
 * @brief Function to handle module ticks.
 * @description Modules Clock is the clock at which this module is working.\
 * @ref module_freq_t for different operational frequencies for this module.\
 * A ms_timer will be running in repeated mode generating pulses with given freq.
 */
void module_tick_handler ();


/*******************Definitions*******************/
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
    led_ui_stop_seq (LED_UI_LOOP_SEQ, LED_SEQ_DETECT_PULSE);
    led_ui_single_start (LED_SEQ_DETECT_WINDOW, LED_UI_MID_PRIORITY, true);
    motion_state = MOTION_SYNC;
    arr_state_change[motion_state] ();
    tssp_detect_pulse_detect ();
    cam_trigger (MOD_MOTION);
}

bool compare_margin(uint32_t data, uint32_t ref, uint32_t margin)
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

bool validate_and_sync (uint32_t ticks)
{
    for(uint32_t freq_cmp = 0; freq_cmp < MAX_MOD_FREQ; freq_cmp++)
    {
        if(compare_margin (ticks, arr_module_tick_duration[freq_cmp], TSSP_DETECT_TICKS_MS(1)))
        {
            return true;
        }
    }
    return false;
}

bool three_window_sync (uint32_t ticks)
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
        tssp_detect_sync_time = pulse_diff_window[1];
        pulse_cnt = PULSE_REQ_FOR_SYNC;
        return (compare_margin (pulse_diff_window[1], pulse_diff_window[0], TSSP_DETECT_TICKS_MS(2))
            && compare_margin (pulse_diff_window[2], pulse_diff_window[1], TSSP_DETECT_TICKS_MS(2))
            && validate_and_sync (pulse_diff_window[1]));
    }
    
    return false;
}

void pulse_detect_handler (uint32_t ticks_count)
{
    if(motion_state == MOTION_SYNC)
    {
        if(three_window_sync (ticks_count))
        {
            tssp_detect_window_sync (tssp_detect_sync_time);
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

void light_check (oper_time_t oper_time, uint32_t light_intensity, uint32_t module)
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
        arr_is_light_ok[module] = true;
    }
    else
    {
        arr_is_light_ok[module] = false;
    }
}

void light_sense_add_ticks (uint32_t interval)
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
            light_check (sensebe_config.tssp_conf.oper_time, light_intensity, MOD_MOTION);
        }
        
        //timer light check
        if(arr_is_light_sense_req[MOD_TIMER])
        {
            light_check (sensebe_config.timer_conf.oper_time, light_intensity, MOD_TIMER);
        }
        
        if(arr_is_light_sense_req[MOD_IR_TX])
        {
            light_check (sensebe_config.ir_tx_conf.oper_time, light_intensity, MOD_IR_TX);
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
        .trig_mode = sensebe_config.tssp_conf.cam_oper.mode,
        .trig_param1 = sensebe_config.tssp_conf.cam_oper.larger_value,
        .trig_param2 = sensebe_config.tssp_conf.cam_oper.smaller_value,
        .pre_focus_en = (bool)sensebe_config.timer_conf.cam_oper.pre_focus,
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
    mod_ticks += arr_module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed];
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
        .trig_mode = sensebe_config.timer_conf.cam_oper.mode,
        .trig_param1 = sensebe_config.timer_conf.cam_oper.larger_value,
        .trig_param2 = sensebe_config.timer_conf.cam_oper.smaller_value,
        .pre_focus_en = (bool)sensebe_config.timer_conf.cam_oper.pre_focus,
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
    mod_ticks += arr_module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed];
    if(mod_ticks >= timer_module_value)
    {
        timer_trigger_handler ();
        mod_ticks = 0;
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
    
    if(sensebe_config.ir_tx_conf.ir_tx_pwr == MAX_IR_RANGES)
    {
        sensebe_config.ir_tx_conf.ir_tx_pwr = IR_LONG;
    }

    tx_range = sensebe_config.ir_tx_conf.ir_tx_pwr;
    arr_ir_range_select[tx_range]();    
    led_ui_single_start (arr_range_indicator[tx_range], LED_UI_LOW_PRIORITY, true);

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
    if(memcmp (&sensebe_config, sensebe_store_config_get_last_config(),
               sizeof(sensebe_config_t)) != 0)
    {
        sensebe_store_config_write (&sensebe_config);
    }

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
        MS_TIMER_TICKS_MS(arr_module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed])
        , module_tick_handler);
    
    light_sense_add_ticks (LIGHT_SENSE_INTERVAL_TICKS);
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
        light_sense_add_ticks (interval);
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
    tx_range = (tx_range + 1)%MAX_IR_RANGES;
    led_ui_single_start (arr_range_indicator[tx_range], LED_UI_LOW_PRIORITY, true);

    arr_ir_range_select[tx_range]();
    sensebe_config.ir_tx_conf.ir_tx_pwr = tx_range;
}
