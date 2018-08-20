/* 
 * File:   sensepi_cam_trigger.c
 * Copyright (c) 2018 Appiko
 * Created on 14 August, 2018, 5:42 PM
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

#include "sensepi_cam_trigger.h"

#include "sensepi_ble.h"

#include "pir_sense.h"
#include "led_sense.h"
#include "mcp4012_x.h"
#include "ms_timer.h"
#include "led_ui.h"
#include "out_pattern_gen.h"
#include "log.h"

#include "hal_pin_analog_input.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"

#include "nrf_util.h"
#include "nrf_assert.h"
#include "common_util.h"

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "boards.h"

#define DEBUG_PRINT 0

/**Delay required when LED is to be used as INPUT*/
#define LED_WAIT_TIME_MS 301
/**Minimum time required to enable PIR sensing*/
#define PIR_SENSE_INTERVAL_MS 50
/** Multiplying factor to convert PIR threshold value received from app to one 
 *  appropriate for module  */
#define PIR_THRESHOLD_MULTIPLY_FACTOR 8
/** Multiplying factor to convert Light threshold value received from app to one 
 *  appropriate for module  */
#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32
/** The time in ms (min*sec*ms) after which light sensing is to be done */
#define LIGHT_SENSE_INTERVAL_MS   (5*60*1000)
/** The time in ms timer ticks after which light sensing is to be done */
#define LIGHT_SENSE_INTERVAL_TICKS (MS_TIMER_TICKS_MS(LIGHT_SENSE_INTERVAL_MS))
/** The time in ms (min*sec*ms) to show feedback with the LED
 *  pulsing in the Sense mode */
#define SENSE_FEEDBACK_TIMEOUT_MS   (9.8*60*1000)
/** The time in ms timer ticks to show feedback with the LED
 *  pulsing in the Sense mode */
#define SENSE_FEEDBACK_TIMEOUT_TICKS (MS_TIMER_TICKS_MS(SENSE_FEEDBACK_TIMEOUT_MS))
/** MS_TIMER used in this module */
#define SENSEPI_CAM_TRIGGER_MS_TIMER_USED MS_TIMER2
/*Data_Process module MACROS*/
/** Number of output pins used */
#define NUM_PIN_OUT 2
/** Size of 1 byte in bits */
#define SIZE_OF_BYTE 8
/** Byte position of MODE in 32 bit uint variable */
#define MODE_POS 0
/** Byte position of INPUT1 in 32 bit uint variable */
#define INPUT1_POS 1
/** Byte position of INPUT2 in 32 bit uint variable */
#define INPUT2_POS 3
/** Mask to separate MODE from rest of the data */
#define MODE_MSK 0x000000FF
/** Mask to separate INPUT1 from rest of the data */
#define INPUT1_MSK 0x00FFFF00
/** Mask to separate INPUT2 from rest of the data */
#define INPUT2_MSK 0xFF000000

/** Number of transitions required for single shot operation */
#define SINGLE_SHOT_TRANSITIONS 2
/** Ticks duration required for single shot operation */
#define SINGLE_SHOT_DURATION MS_TIMER_TICKS_MS(250)
/** Number of transitions required for bulb operation */
#define BULB_SHOT_TRANSITIONS 3
/** Ticks duration of trigger pulse at the end of operation */
#define BULB_TRIGGER_PULSE MS_TIMER_TICKS_MS(250)
/** Number of transitions required for focus operation */
#define FOCUS_TRANSITIONS 2

/** Copy of configuration to be shared across module */
static sensepi_cam_trigger_init_config_t config;
/** PIR configuration which will be used to enable PIR */
static pir_sense_cfg config_pir;
/** Array of out_gen_config_t to store pre-calculated configs */
static out_gen_config_t out_gen_config[2];
/** Flag to keep status of PIR's expected state */
static bool pir_on_flag = false;
/** The amount of time since starting the SENSING state */
static uint32_t sense_count;
/** Boolen to indicate if PIR sensing feedback is required for user  */
static bool sense_feedback = false;

/***/
static const bool out_gen_end_all_on[OUT_GEN_MAX_NUM_OUT] = {1,1,1,1};
/***/
static const bool multishot_generic[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS] =
    {
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}
    };

/**
 * @brief Enum to decide the Mode for data processing. 
 */
typedef enum 
{
    /** Do data process according to PIR configuration */
    PIR_DATA_PROCESS_MODE,
    /** Do data process according to TIMER configuration */
    TIMER_DATA_PROCESS_MODE,
}
data_process_mode_t;

/**
 * @brief Enum of different types of camera triggering.
 */
typedef enum 
{
    /** Click single shot per trigger */
    MODE_SINGLE_SHOT,
    /** Click multiple shot per trigger */
    MODE_MULTISHOT,
    /** Click shot in bulb mode */
    MODE_BULB,
    /** Shoot video with start and stop pulse */
    MODE_VIDEO,
    /** Focus only */
    MODE_FOCUS,
    /** Mode dedicated to do nothing */
    MODE_NONE,
} 
operational_mode_t;

/** 
 * @brief Enum of different states of camera trigger
 */
typedef enum
{
    PIR_IDLE,
    TIMER_IDLE,
}
cam_trig_state_t;

/*PIR related functions*/
/**
 * @brief Function to enable or disable PIR according to conditions
 * @param state if true enable the PIR if false disable the PIR
 */
void pir_set_state(bool state);
/**
 * @brief IRQ Handler for PIR interrupt
 */
void pir_handler(int32_t adc_val);
/**
 * @brief Function to update the out_gen_configs for PIR.
 */
void pir_out_gen_config_updater(void);

/*Timer related functions*/
/**
 * @brief Function to enable or disable timer according to conditions
 * @param state if true enable the timer if false disable the timer
 */
void timer_set_state(bool state);
/**
 * @brief IRQ Handler for periodic TIMER interrupt
 */
void timer_handler(void);
/**
 * @brief Function to update the out_gen_configs for Timer.
 */
void timer_out_gen_config_updater(void);

/*out_pattern_gen related functions*/
/**
 * @brief Handler for out pattern generation done.
 * @param out_gen_state State from which handler is called.
 */
void pattern_out_done_handler(uint32_t out_gen_state);

/*Light Sense related functions*/
/**
 * @brief Function to get required data to initialize the led_sense module
 */
void light_sense_init(void);
/**
 * @brief Function to enable or disable light_sense according to conditions
 * @param state if true enable the light_sense if false disable the light_sense
 */
void light_sense_set_state(bool state);
/**
 * @brief Function which will check light intensity and compare it to 
 * current operational mode sent as parameter.
 * @param oper_time value for which we need to check light conditions
 * @return true if light intensity satisfies given conditions
 * @return false if light intensity doesn't satisfies given condition
 */
bool light_sense_light_check(oper_time_t oper_time);

/*Module manager related function*/
//Module manager is part of program which will enable or disable the 
//PIR or timer at start event, add_ticks event and stop event
/**
 * @brief Function to check light and working mode conditions.
 * according to conditions respective peripheral device will be
 * enabled or disabled.
 */
void module_manager_start_check(void);
/**
 * @brief Function to disable all peripheral device used in this module.
 */
void module_manager_disable_all(void);

/**
 * @brief Functions to generate out_gen_config according to selected camera
 * trigger mode.
 * @{
 */
/**
 * @brief Function to click one shot per trigger
 * @param data_process_mode mode for which we have to generate the out_gen_config
 */
void single_shot_mode(data_process_mode_t data_process_mode);
/**
 * @brief Function to click multiple shots per trigger
 * @param data_process_mode mode for which we have to generate the out_gen_config
 * @param burst_duration Time duration between two shots
 * @param burst_num Number of shots per trigger
 */
void multi_shot_mode(data_process_mode_t data_process_mode, 
                     uint32_t burst_duration, uint32_t burst_num);
/**
 * @brief Function to take photo in bulb mode.
 * @param data_process_mode mode for which we have to generate the out_gen_config
 * @param bulb_time Time duration for half press signal
 */
void bulb_mode(data_process_mode_t data_process_mode, uint32_t bulb_time);
/**
 * @brief Function to do focus only
 * @param data_process_mode mode for which we have to generate the out_gen_config
 */
void focus_mode(data_process_mode_t data_process_mode);
/**
 * @brief Dummy mode which will do nothing i.e keep the signals high
 * @param data_process_mode mode for which we have to generate the out_gen_config
 */
void none_mode(data_process_mode_t data_process_mode);

//function definitions
/**
 * @brief Function to print expected pattern output from out_gen_pattern
 * @param next Array of expected states
 * @param str Comment to describe for which we are printing the array.
 */
void debug_print_bool_array(bool (* next)[OUT_GEN_MAX_TRANSITIONS], char * str)
{
#if DEBUG_PRINT
    log_printf("%s\n", str);
    for(uint32_t row = 0; row< NUM_PIN_OUT; row++)
    {
        for(uint32_t arr_p = 0; arr_p<OUT_GEN_MAX_TRANSITIONS; arr_p++)
        {
            log_printf("%x ", next[row][arr_p]);
        }
        log_printf("\n");
    }
    log_printf("\n");
#endif
}

void pir_set_state(bool state)
{
    if(state == true)
    {
        pir_sense_start(&config_pir);        
    }
    else
    {
        pir_sense_stop();    
    }
}

void pir_handler(int32_t adc_val)
{
    log_printf("%s", __func__);
    log_printf(" %d\n", adc_val);
    if(out_gen_is_on() == false)
    {
        if(sense_feedback == true)
        {
            led_ui_single_start(LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
        }
        out_gen_start(&out_gen_config[PIR_DATA_PROCESS_MODE]);
    }
}

void pir_out_gen_config_updater()
{
    uint32_t config_mode = config.config_sensepi->pir_conf.mode;
    uint32_t mode = ((config_mode & MODE_MSK) 
            >> (MODE_POS * SIZE_OF_BYTE));
    uint32_t input1 = ((config_mode & INPUT1_MSK) 
            >> (INPUT1_POS * SIZE_OF_BYTE));
    uint32_t input2 = ((config_mode & INPUT2_MSK) 
            >> (INPUT2_POS * SIZE_OF_BYTE));
    uint32_t single_input = config_mode>> (INPUT1_POS * SIZE_OF_BYTE);
    switch((operational_mode_t)mode)
    {
        case MODE_SINGLE_SHOT:
        {
            single_shot_mode(PIR_DATA_PROCESS_MODE);
            break;
        }
        case MODE_MULTISHOT:
        {
            multi_shot_mode(PIR_DATA_PROCESS_MODE, input1, input2);
            break;
        }
        case MODE_BULB :
        {
            //Using both input 1 and input 2
            bulb_mode(PIR_DATA_PROCESS_MODE, single_input);
            break;
        }
        case MODE_VIDEO :
        {
            //Using both input 1 and input 2
            break;
        }
        case MODE_FOCUS :
        {
            focus_mode(PIR_DATA_PROCESS_MODE);
            break;
        }
        case MODE_NONE:
        {
            none_mode(PIR_DATA_PROCESS_MODE);
        }
    }   
}

void timer_set_state(bool state)
{
    if(state == true)
    {
        if(ms_timer_get_on_status (SENSEPI_CAM_TRIGGER_MS_TIMER_USED) == false)
        {
            ms_timer_start(SENSEPI_CAM_TRIGGER_MS_TIMER_USED, MS_REPEATED_CALL,
               MS_TIMER_TICKS_MS(config.config_sensepi->timer_conf.timer_interval * 100),
               timer_handler);
        }
    }
    else
    {
        ms_timer_stop(SENSEPI_CAM_TRIGGER_MS_TIMER_USED);
    }
}

void timer_handler()
{
    if(out_gen_is_on () == false)
    {
        out_gen_start(&out_gen_config[TIMER_DATA_PROCESS_MODE]);
    }
}

void timer_out_gen_config_updater(void)
{
    uint32_t config_mode = config.config_sensepi->timer_conf.mode;
    uint32_t mode = ((config_mode & MODE_MSK) 
            >> (MODE_POS * SIZE_OF_BYTE));
    uint32_t input1 = ((config_mode & INPUT1_MSK) 
            >> (INPUT1_POS * SIZE_OF_BYTE));
    uint32_t input2 = ((config_mode & INPUT2_MSK) 
            >> (INPUT2_POS * SIZE_OF_BYTE));
    uint32_t single_input = config_mode>> (INPUT1_POS * SIZE_OF_BYTE);
    switch((operational_mode_t)mode)
    {
        case MODE_SINGLE_SHOT:
        {
            single_shot_mode(TIMER_DATA_PROCESS_MODE);
            break;
        }
        case MODE_MULTISHOT:
        {
            multi_shot_mode(TIMER_DATA_PROCESS_MODE, input1, input2);
            break;
        }
        case MODE_BULB :
        {
            //Using both input 1 and input 2
            bulb_mode(TIMER_DATA_PROCESS_MODE, single_input);
            break;
        }
        case MODE_VIDEO :
        {
            //Using both input 1 and input 2
            break;
        }
        case MODE_FOCUS :
        {
            focus_mode(TIMER_DATA_PROCESS_MODE);
            break;
        }
        case MODE_NONE:
        {
            none_mode(PIR_DATA_PROCESS_MODE);
        }
    }   
}

void pattern_out_done_handler (uint32_t out_gen_state)
{
    log_printf("%s",__func__);
    log_printf(" %d\n", out_gen_state);
    switch((cam_trig_state_t)out_gen_state)
    {
        case PIR_IDLE :
        {
            pir_set_state(pir_on_flag);
            break;
        }
        case TIMER_IDLE : 
        {
            break;
        }
    }
}

void light_sense_init(void)
{
    log_printf("%s\n",__func__);
    led_sense_init (config.led_sense_out_pin,
                    config.led_sense_analog_in_pin,
                    config.led_sense_off_val);
}

void light_sense_set_state(bool state)
{
    if(state == true)
    {
        led_sense_cfg_input(true);
        //To make sure that the green LED is ready to sense light
        hal_nop_delay_ms(LED_WAIT_TIME_MS);
    }
    else
    {
        led_sense_cfg_input(false);
    }
}

bool light_sense_light_check(oper_time_t oper_time)
{
    log_printf("%d\n", __func__);

    uint8_t light_sense_config = oper_time.day_or_night;
    uint32_t light_threshold =
            (uint32_t)((oper_time.threshold) * LIGHT_THRESHOLD_MULTIPLY_FACTOR);

    uint32_t light_intensity = led_sense_get();
    log_printf("Light Intensity : %d\n", light_intensity);

    static bool light_check_flag = 0;
    //Day and its brighter than the threshold
    if(((light_sense_config == 1) && (light_intensity >= light_threshold))
            ||  //Night and its dimmer than the threshold
       ((light_sense_config == 0) && (light_intensity <= light_threshold)))
    {
        light_check_flag = 1;
    }
    else
    {
        light_check_flag = 0;
    }
    return light_check_flag;
}

void module_manager_start_check(void)
{
    log_printf("%s\n", __func__);
    //Switch off PIR module so that light sensing can happen
    pir_set_state(false);
    bool light_flag;
    switch(config.config_sensepi->trig_conf)
    {
        case PIR_ONLY : 
        {
            light_flag = light_sense_light_check(config.config_sensepi->pir_conf.oper_time);
            pir_on_flag = light_flag;
            pir_set_state(light_flag);
            
            break;
        }
        case TIMER_ONLY : 
        {
            light_flag = light_sense_light_check(config.config_sensepi->timer_conf.oper_time);
            timer_set_state(light_flag);
                    
            break;
        }
        case PIR_AND_TIMER :
        {
            light_flag = light_sense_light_check(config.config_sensepi->pir_conf.oper_time);
            pir_on_flag = light_flag;
            pir_set_state(light_flag);

            light_flag = light_sense_light_check(config.config_sensepi->timer_conf.oper_time);
            timer_set_state(light_flag);

            break;
        }
    }

}

void module_manager_disable_all(void)
{
    pir_set_state(false);
    timer_set_state(false);
    light_sense_set_state(false);
}

void single_shot_mode(data_process_mode_t data_process_mode)
{
    log_printf("%s\n", __func__);
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    uint32_t local_out_gen_state, time_remain;

    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100) 
            - (SINGLE_SHOT_DURATION); 
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
        local_out_gen_state = PIR_IDLE;
    }
    else
    {
        local_out_gen_state = TIMER_IDLE;
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = local_out_gen_state,
        .transitions_durations = { SINGLE_SHOT_DURATION, time_remain },
        .next_out = {{0, 1, 1},
                     {0, 1, 1}},
    };

    debug_print_bool_array(local_out_gen_config.next_out, "single shot");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void multi_shot_mode(data_process_mode_t data_process_mode, 
                     uint32_t burst_duration, uint32_t burst_num)
{
    log_printf("%s\n", __func__);
    uint32_t local_out_gen_state, time_remain;
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS * burst_num;
    //Time for trigger pulse and time till next trigger for each burst
    uint32_t repeat_delay_array[SINGLE_SHOT_TRANSITIONS] = {SINGLE_SHOT_DURATION,
            MS_TIMER_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION};
    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = pattern_out_done_handler,
    };

    for(uint32_t i = 0; i< burst_num; i++)
    {
        memcpy(local_out_gen_config.transitions_durations + i*SINGLE_SHOT_TRANSITIONS,
                repeat_delay_array, SINGLE_SHOT_TRANSITIONS*sizeof(uint32_t));

        for(uint32_t j = 0; j < NUM_PIN_OUT; j++)
        {
            memcpy(*(local_out_gen_config.next_out +j),*(multishot_generic + j),
                    burst_num* SINGLE_SHOT_TRANSITIONS*sizeof(bool));
            //An extra '1' at the end for the remaining of the inter-trigger time
            local_out_gen_config.next_out[j][burst_num* SINGLE_SHOT_TRANSITIONS] = 1;
        }
    }

    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - SINGLE_SHOT_DURATION*burst_num -
            (MS_TIMER_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION)*(burst_num - 1);  
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
        local_out_gen_state = PIR_IDLE;
   }
    else
    {
        time_remain = 1;
        local_out_gen_state = TIMER_IDLE;
    }
    local_out_gen_config.out_gen_state = local_out_gen_state;
    // Last interval for the '1' signal till 'time till next trigger' elapses
    local_out_gen_config.transitions_durations[number_of_transition-1] = time_remain;
            

    debug_print_bool_array(local_out_gen_config.next_out, "multi shot");
    memcpy(&out_gen_config[data_process_mode], &local_out_gen_config,
           sizeof(out_gen_config_t));
}

void bulb_mode(data_process_mode_t data_process_mode, uint32_t bulb_time)
{
    log_printf("%s\n", __func__);
    uint32_t local_out_gen_state, time_remain;
    uint32_t number_of_transition = BULB_SHOT_TRANSITIONS;
    uint32_t bulb_time_ticks = MS_TIMER_TICKS_MS((bulb_time*100));

    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - bulb_time_ticks ;
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
        local_out_gen_state = PIR_IDLE;
    }
    else
    {
        time_remain = 1;
        local_out_gen_state = TIMER_IDLE;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = local_out_gen_state,
        .transitions_durations = 
            {bulb_time_ticks - BULB_TRIGGER_PULSE, BULB_TRIGGER_PULSE
              , time_remain},
        .next_out = { {0, 0, 1, 1},
                      {1, 0, 1, 1} },
    };

    debug_print_bool_array(local_out_gen_config.next_out, "bulb mode");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void focus_mode(data_process_mode_t data_process_mode)
{
    log_printf("%s\n", __func__);
    uint32_t local_out_gen_state, time_remain;
    uint32_t number_of_transition = FOCUS_TRANSITIONS;

    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        time_remain =  MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - SINGLE_SHOT_DURATION ;
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
        local_out_gen_state = PIR_IDLE;
    }
    else
    {
        time_remain = 1;
        local_out_gen_state = TIMER_IDLE;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = local_out_gen_state,
        .transitions_durations =
        {SINGLE_SHOT_DURATION ,time_remain},
        .next_out = { {0, 1, 1},
                      {1, 1, 1} },
    };
    
    debug_print_bool_array(local_out_gen_config.next_out, "focus mode");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void none_mode(data_process_mode_t data_process_mode)
{
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = 1,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = data_process_mode,
        .transitions_durations = {1},
        .next_out = { {1},
                      {1} },
    };

    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void sensepi_cam_trigger_init(sensepi_cam_trigger_init_config_t * config_sensepi_cam_trigger)
{
    log_printf("%s\n", __func__);
    ASSERT(config_sensepi_cam_trigger->signal_pin_num == NUM_PIN_OUT);
    memcpy(&config, config_sensepi_cam_trigger, sizeof(config));
    light_sense_init();
    mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
    pir_sense_cfg local_config_pir = 
    {
        PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
        config.pir_sense_offset_input,
        ((uint32_t)config.config_sensepi->pir_conf.threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
        APP_IRQ_PRIORITY_HIGH, pir_handler, 
    };
    memcpy(&config_pir, &local_config_pir, sizeof(pir_sense_cfg));
    out_gen_init(NUM_PIN_OUT, config.signal_out_pin_array, 
                 (bool *) out_gen_end_all_on);
}

void sensepi_cam_trigger_update(sensepi_config_t * update_config)
{
    log_printf("%s\n", __func__);
    memcpy(config.config_sensepi, update_config, sizeof(sensepi_config_t));
}

void sensepi_cam_trigger_start()
{
    log_printf("%s\n", __func__);

    sense_count = 0;
    sense_feedback = true;

    config_pir.threshold = ((uint32_t) config.config_sensepi->pir_conf.threshold)
            *PIR_THRESHOLD_MULTIPLY_FACTOR;

    mcp4012_set_value(config.config_sensepi->pir_conf.amplification);

    pir_out_gen_config_updater();
    timer_out_gen_config_updater();

    light_sense_set_state(true);
    module_manager_start_check();
}

void sensepi_cam_trigger_add_tick(uint32_t interval)
{
    static uint32_t light_sense_count = 0;

    log_printf("%s\n", __func__);
    log_printf("SensePi Add ticks : %d\n", interval);

    sense_count += interval;
    if(sense_count > SENSE_FEEDBACK_TIMEOUT_TICKS)
    {
        sense_feedback = false;
    }

    light_sense_count += interval;
    if(light_sense_count > LIGHT_SENSE_INTERVAL_TICKS)
    {
        module_manager_start_check();
        light_sense_count = (light_sense_count - LIGHT_SENSE_INTERVAL_TICKS);
    }
}

void sensepi_cam_trigger_stop()
{
    log_printf("%s\n",__func__);
    module_manager_disable_all();
    out_gen_stop((bool *) out_gen_end_all_on);
    led_ui_type_stop_all(LED_UI_SINGLE_SEQ);
}

sensepi_config_t * sensepi_cam_trigger_get_sensepi_config()
{
    log_printf("%s\n", __func__);
    return (config.config_sensepi);
}
