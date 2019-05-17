/*
 *  sensebe_tx_rx_mod.c : Module to handle SenseBe's Tx Rx functionalities
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


#include "sensebe_ble.h"
#include "sensebe_tx_mod.h"
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
#include "radio_trigger.h"

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
#define MOTION_SYNC_ON_TIME 200
/** Off time for TSSP receiver while module is in motion sync mode */
#define MOTION_SYNC_OFF_TIME 800

#define RADIO_ON_FREQ_MS 50

/***********ENUMS***********/
/** List of all submodules which are being managed by this module */
typedef enum
{
    /**Timer Module*/
    MOD_TIMER,
    /**Radio Module*/
    MOD_RADIO,
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
    MOD_FREQ0 = (5),
    /**Wake up time 25ms*/
    MOD_FREQ1 = (25),
    /**Wake up time 50ms*/
    MOD_FREQ2 = (50),
    /**Wake up time 100ms*/
    MOD_FREQ3 = (100),
    /**Maximum number of frequencies possible*/
    MAX_MOD_FREQ = 4,
}module_freq_t;

/** List of HW modes */
typedef enum 
{
    /** Enable TX HW circuitry */
    TX_EN = SENSEBE_TX_BOARD,
    /** Enable RX HW circuitry  */
    RX_EN = SENSEBE_RX_BOARD
}rx_tx_mod_en_t;

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

static uint32_t radio_mod_ticks_freq = 0;

/***********FUNCTIONS***********/
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

void radio_module_start ();

void radio_module_add_mod_ticks ();

void radio_module_stop ();

void radio_module_trigger_handler (void * p_trig, uint32_t len);

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

void radio_module_start ()
{
    arr_is_mod_on[MOD_RADIO] = 1;
    radio_trigger_init_t radio_init = 
    {
        .comm_direction = RADIO_TRIGGER_Rx,
        .comm_freq = 95,
        .irq_priority = APP_IRQ_PRIORITY_HIGH,
        .radio_trigger_rx_callback = radio_module_trigger_handler,
        .rx_on_time_ms = 1,
    };
    radio_trigger_init (&radio_init);
    
    cam_trigger_config_t cam_trigg = 
    {
        .setup_number = MOD_RADIO,
        .pre_focus_en = sensebe_config.cam_trigs[RADIO_ALL].pre_focus,
        .trig_duration_100ms = 15,
        .trig_mode = sensebe_config.cam_trigs[RADIO_ALL].mode,
        .trig_param1 = sensebe_config.cam_trigs[RADIO_ALL].larger_value,
        .trig_param2 = sensebe_config.cam_trigs[RADIO_ALL].smaller_value,
    };
    cam_trigger_set_trigger (&cam_trigg);
    
}

void radio_module_add_mod_ticks ()
{
    static uint32_t radio_cnt = 0;
    radio_cnt++;
    if(radio_cnt >= radio_mod_ticks_freq)
    {
        radio_trigger_listen ();
        radio_cnt = 0;
    }
}

void radio_module_stop ()
{
    arr_is_mod_on[MOD_RADIO] = 0;
    radio_trigger_shut ();
}

void radio_module_trigger_handler (void * p_trig, uint32_t len)
{
    uint32_t * buff = (uint32_t *)p_trig;
    log_printf("%s : %d\n", __func__, buff[0]);
    cam_trigger (buff[0]);
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
        .trig_mode = sensebe_config.cam_trigs[TIMER_ALL].mode,
        .trig_param1 = sensebe_config.cam_trigs[TIMER_ALL].larger_value,
        .trig_param2 = sensebe_config.cam_trigs[TIMER_ALL].smaller_value,
        .pre_focus_en = (bool)sensebe_config.cam_trigs[TIMER_ALL].pre_focus,
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
    if(arr_is_mod_on[MOD_IR_TX] == true)
    {
        ir_tx_module_add_mod_ticks ();
    }
    if(arr_is_mod_on[MOD_RADIO] == true)
    {
        radio_module_add_mod_ticks ();
    }
}

void sensebe_tx_rx_init (sensebe_tx_config_t * sensebe_tx_init)
{
    log_printf("%s\n", __func__);
        
    //Assign Enable and sense pins
    light_check_sense_pin = sensebe_tx_init->light_sense_config.photodiode_pin;
    hal_gpio_cfg_input (light_check_sense_pin, HAL_GPIO_PULL_DOWN);
    light_check_en_pin = sensebe_tx_init->light_sense_config.photodiode_en_pin;
    hal_gpio_cfg_output (light_check_en_pin, 0);
    

    
    memcpy (&sensebe_config, sensebe_tx_init->sensebe_config,
            sizeof(sensebe_config_t));
        
    cam_trigger_setup_t cam_trig_setup = 
    {
        .cam_trigger_done_handler = camera_unit_handler,
        .focus_pin = sensebe_tx_init->cam_config.focus_pin_no,
        .trigger_pin = sensebe_tx_init->cam_config.trigger_pin_no
    };
    cam_trigger_init (&cam_trig_setup);    
    
    tssp_ir_tx_init (sensebe_tx_init->tx_transmit_config.tx_en_pin,
                     sensebe_tx_init->tx_transmit_config.tx_in_pin);
    
    ir_pwr1 = sensebe_tx_init->tx_transmit_config.tx_pwr1;
    ir_pwr2 = sensebe_tx_init->tx_transmit_config.tx_pwr2;
    

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
    
    if((sensebe_config.trig_conf != MOTION_ONLY))
    {
        timer_module_start ();
    }
    else
    {
        timer_module_stop ();
    }
    
    if ((sensebe_config.trig_conf != TIMER_ONLY))
    {
        radio_module_start ();
    }
    else
    {
        radio_module_stop ();
    }

    if ((sensebe_config.ir_tx_conf.is_enable == 1))
    {
        ir_tx_module_start ();
    }
    else
    {
        ir_tx_module_stop ();
    }
    
    radio_mod_ticks_freq = RADIO_ON_FREQ_MS/
        arr_module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed];
    
    ms_timer_start (SENSEBE_OPERATION_MS_TIMER, MS_REPEATED_CALL,
        MS_TIMER_TICKS_MS(arr_module_tick_duration[sensebe_config.ir_tx_conf.ir_tx_speed])
        , module_tick_handler);
    
    light_sense_add_ticks (LIGHT_SENSE_INTERVAL_TICKS);
}

void sensebe_tx_rx_stop (void)
{
    log_printf("%s\n", __func__);
    cam_trigger_stop ();
    timer_module_stop ();
    ir_tx_module_stop ();
    radio_module_stop ();
    ms_timer_stop (SENSEBE_OPERATION_MS_TIMER);
}

void sensebe_tx_rx_add_ticks (uint32_t interval)
{
    add_ticks_feedback (interval);

    if(arr_is_light_sense_req [MOD_TIMER] == true ||
       arr_is_light_sense_req [MOD_IR_TX] == true)
    {
        light_sense_add_ticks (interval);
    }
    
    if((sensebe_config.trig_conf != MOTION_ONLY))
    {
        timer_module_add_ticks ();
    }
    if((sensebe_config.ir_tx_conf.is_enable == 1))
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
