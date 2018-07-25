/* 
 * File:   sensepi_cam_trigger.c
 * Copyright (c) 2018 Appiko
 * Created on 1 June, 2018, 3:30 PM
 * Author: Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
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
#include "log.h"
#include "device_tick.h"
#include "mcp4012_x.h"
#include "ms_timer.h"
#include "out_pattern_gen.h"

#include "boards.h"

#include "hal_pin_analog_input.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "common_util.h"

#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "nrf_assert.h"

/*Make this 1 while doing debugging over UART*/
#define DEBUG_PRINT 1

/*Sensepi_PIR module MACROS*/
/**Delay required when LED is to be used as INPUT*/
#define LED_WAIT_TIME_MS 301
/**Minimum time required to enable PIR sensing*/
#define PIR_SENSE_INTERVAL_MS 50
/***/
#define PIR_SENSE_THRESHOLD 600
/** Multiplying factor to convert PIR threshold value received from app to one 
 *  appropriate for module
 */
#define PIR_THRESHOLD_MULTIPLY_FACTOR 8
/** Multiplying factor to convert Light threshold value received from app to one 
 *  appropriate for module
 */
#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32

#define SENSEPI_CAM_TRIGGER_MS_TIMER_USED MS_TIMER2
/*Data_Process module MACROS*/

#define SIZE_OF_BYTE 8
#define POS_OF_MODE 0
#define POS_OF_INPUT1 1
#define POS_OF_INPUT2 3
#define MODE_MSK 0x000000FF
#define INPUT1_MSK 0x00FFFF00
#define INPUT2_MSK 0xFF000000

#define NUM_PIN_OUT 2
#define FOCUS_TRIGGER_TIME_DIFF LFCLK_TICKS_MS(20)
#define SINGLE_SHOT_TRANSITIONS 2
#define BULB_SHOT_TRANSITIONS 3
#define SINGLE_SHOT_DURATION LFCLK_TICKS_MS(250)
#define BULB_TRIGGER_PULSE LFCLK_TICKS_MS(250)
#define VIDEO_TRANSITION 2
#define VIDEO_CONTROL_PULSE LFCLK_TICKS_MS(250)
#define FOCUS_TRANSITIONS 2


/*SensePi_PIR module variables.*/
/**Copy of configuration to be shared across module*/
static sensepi_cam_trigger_init_config_t config;
/**PIR configuration which will be used to enable PIR*/
static pir_sense_cfg config_pir;
/**minimum inter-trigger time between two successive trigger*/
static uint32_t intr_trig_time_in = 0;
/**Timer duration after which timer interrupt will occur*/
static uint32_t timer_interval_in = 0;
/**Extension time which has to be added after PIR interrupt occur*/
static uint32_t video_ext_time = 0;
/**Flag to keep status of video*/
static bool video_on_flag = false;
/**Flag to keep status of PIR operation*/
volatile bool pir_oper_flag = false;
/**Flag to keep status of PIR's current state*/
static bool pir_on_flag = false;
/**Flag to keep status of TIMER's current state*/
static bool timer_on_flag = false;

/**Time remain in video length*/
static uint32_t time_remain = 0;
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
static void (*out_gen_done_ptr)(uint32_t out_gen_ptr_state);

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
    /** Clicl shot in bulb mode */
    MODE_BULB,
    /** Click video according to configuration */
    MODE_VIDEO,
    /** Focus only */
    MODE_FOCUS,
} 
operational_mode_t;

/** 
 * @brief Enum of different states of camera trigger
 */
typedef enum
{
    IDLE,
    VIDEO_PIR_ON,
    VIDEO_END,
    TIMER_IDLE,
}
cam_trig_state_t;

/**
 * @brief Local function used to start the operation of module internally.
 */
void start_sense();

/**
 * @brief Function to enable PIR sensing
 */
void pir_enable();

/**
 * @brief Function to disable PIR sensing
 */
void pir_disable();

/**
 * @brief IRQ Handler for PIR interrupt
 */
void pir_handler(int32_t adc_val);
/**
 * @brief Function to enable timer
 */
void timer_enable();

/**
 * @brief Function to disable timer
 */
void timer_disable();

/**
 * @brief IRQ Handler for TIMER interrupt
 */
void timer_handler(void);

/**
 * @brief Function to enable LED sensing with proper parameters.
 * @param led_conf Configuration which is to be used to activate LED light 
 * sensing
 */
void led_sense_conf(sensepi_cam_trigger_init_config_t * led_conf);

/**
 * @brief Function to check light conditions and comapre light conditions with 
 * configuration provided by user.
 * @param oper_time_temp Local copy of oper_time to select light condition
 * configuration
 * @return if light conditions satisfies the conditions provided by config
 * return 1, else retunr 0.
 */
bool light_check(oper_time_t oper_time_temp);

/**
 * @brief Function which is to be called to generate and send output pattern
 * @param data_process_mode boolean value to select from which configuration
 * mode has to be selected
 */
void data_process_pattern_gen(bool data_process_mode);

/**
 * @brief Function to get extension value for video in case of PIR trigger.
 * @return Extention time
 */
uint32_t data_process_get_video_extention(void);

/**
 * @brief Functions to generate patterns according to selected camera trigger
 * mode.
 * @{
 */
/**
 * @brief Function to click one shot per trigger
 */
void single_shot_mode();
/**
 * @brief Function to click multiple shots per trigger
 * @param burst_duration Time duration between two shots
 * @param burst_num Number of shots per trigger
 */
void multi_shot_mode(uint32_t burst_duration, uint32_t burst_num);
/**
 * @brief Function to take photo in bulb mode.
 * @param bulb_time Time duration for half press signal
 */
void bulb_mode(uint32_t bulb_time);
/**
 * @brief Function to take video according to configuration
 * @param video_time Minimum Duration for footage.
 */
void video_mode(uint32_t video_time);
/**
 * @brief Function to do focus only
 */
void focus_mode();
/**
 * @}
 */

/**
 * @brief Handler for out pattern generation done.
 * @param out_gen_state State from which handler is called.
 */
void pattern_out_done_handler(uint32_t out_gen_state);

///Function definitions

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

/*SensePi_PIR module*/
void pir_enable()
{
    log_printf("PIR_Enabled\n");
    if(pir_on_flag == false)
    {
        pir_on_flag = true;
        log_printf("%d, %d, %d, %d, %x, %x\n", config_pir.sense_interval_ms,
                config_pir.pir_signal_analog_in, config_pir.pir_offset_analog_in,
                config_pir.threshold, config_pir.irq_priority, config_pir.handler);
        pir_sense_start(&config_pir);
    }
}

void pir_disable()
{
    log_printf("PIR_Disabled\n");
    pir_on_flag = false;
    pir_sense_stop();
}

void pir_handler(int32_t adc_val)
{
    log_printf("PIR Handler\n");
    log_printf("Sensed %d\n", adc_val);
//    while(out_gen_is_on() == true)
    {
//        do_nothing
    }
    pir_disable();
    pir_oper_flag = true;
    out_gen_stop((bool *) out_gen_end_all_on);
    if(video_on_flag == true && time_remain > 0)
    {
        time_remain -= out_gen_get_ticks();
        out_gen_config_t video_end_out_gen_config = 
        {
            .num_transitions = 1,
            .out_gen_done_handler = pattern_out_done_handler,
            .out_gen_state = VIDEO_PIR_ON,
            .transitions_durations = {video_ext_time},
            .next_out =  {{1,1},
            {1,1}},
            
        };
        out_gen_start(&video_end_out_gen_config);

        debug_print_bool_array(video_end_out_gen_config.next_out, "Pir handler");
    }
    else
    {
        data_process_pattern_gen(PIR_DATA_PROCESS_MODE);        
    }
}

void timer_enable()
{
    log_printf("TIMER_Enabled\n");
    if(timer_on_flag == false)
    {
        timer_on_flag = true;
        ms_timer_start(SENSEPI_CAM_TRIGGER_MS_TIMER_USED,
            MS_REPEATED_CALL, LFCLK_TICKS_MS(timer_interval_in), timer_handler);
    }
}

void timer_disable()
{
    log_printf("TIMER_Disabled\n");
    timer_on_flag = false;
    ms_timer_stop(SENSEPI_CAM_TRIGGER_MS_TIMER_USED);
}

void timer_handler(void)
{
    log_printf("Timer Handler\n");
    if(((pir_oper_flag == false) && (out_gen_is_on() == false)) == true)
    {
        data_process_pattern_gen(TIMER_DATA_PROCESS_MODE);    
    }
}

void led_sense_conf(sensepi_cam_trigger_init_config_t * led_conf)
{
    log_printf("Led_Sense_Conf\n");
    led_sense_init(led_conf->led_sense_out_pin,
            led_conf->led_sense_analog_in_pin,
            led_conf->led_sense_off_val);
}

bool light_check(oper_time_t oper_time_temp)
{
    log_printf("Light intensity : %d\n", led_sense_get());
    static uint8_t light_sense_config = 1;
    static uint32_t light_threshold = 0;
    static uint32_t light_intensity = 0;
    light_intensity = led_sense_get();
    static bool light_check_flag = 0;
    light_sense_config = oper_time_temp.day_or_night;
    light_threshold = (uint32_t)((oper_time_temp.threshold) * LIGHT_THRESHOLD_MULTIPLY_FACTOR);
    if(light_sense_config == 1 && light_intensity >= light_threshold)
    {
        light_check_flag = 1;
    }
    else if(light_sense_config == 0 && light_intensity <= light_threshold)
    {
        light_check_flag = 1;
    }
    else
    {
        light_check_flag = 0;
    }
    return light_check_flag;
}


void start_sense()
{
    pir_on_flag = false;
    pir_oper_flag = false;
    video_on_flag = false;
    timer_on_flag = false;
    uint32_t working_mode = 0;
    working_mode = config.config_sensepi->trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            if(light_check(config.config_sensepi->pir_conf.oper_time) == true)
            {
                pir_enable();
            }
            else
            {
                pir_disable();
                out_gen_stop((bool *) out_gen_end_all_on);
            }
            break;
        }
        case TIMER_ONLY:
        {
            pir_oper_flag = false;
            if(light_check(config.config_sensepi->timer_conf.oper_time) == true)
            {
                timer_enable();
            }
            else
            {
                timer_disable();
                out_gen_stop((bool *) out_gen_end_all_on);
            }
            break;
        }
        //TODO make an internal config for the oper time
        case PIR_AND_TIMER:
        {
            if(light_check(config.config_sensepi->pir_conf.oper_time) == true)
            {
                pir_enable();
            }
            else
            {
                pir_disable();
                out_gen_stop((bool *) out_gen_end_all_on);
            }
            if(light_check(config.config_sensepi->timer_conf.oper_time) == true)
            {
                timer_enable();
            }
            else
            {
                out_gen_stop((bool *) out_gen_end_all_on);
                timer_disable();
            }
            break;
        }

    }

}

void sensepi_cam_trigger_start()
{
    log_printf("SensePi Cam Trigger Start\n");
    led_sense_cfg_input(true);
    hal_nop_delay_ms(LED_WAIT_TIME_MS);
    
    //This if will check if module is already working or not.

    start_sense();
}

void sensepi_cam_trigger_stop()
{
    log_printf("SensePi Cam Trigger Stop\n");
    led_sense_cfg_input(false);
    pir_disable();
    timer_disable();
    out_gen_stop((bool *) out_gen_end_all_on);
}

void sensepi_cam_trigger_init(sensepi_cam_trigger_init_config_t * config_sensepi_cam_trigger)
{
    uint32_t working_mode = 0;
    log_printf("SensePi Cam Trigger Init\n");
    ASSERT(config_sensepi_cam_trigger->signal_pin_num == NUM_PIN_OUT);
    memcpy(&config, config_sensepi_cam_trigger, sizeof(config));
    sensepi_config_t local_sensepi_config_t;
    memcpy(&local_sensepi_config_t, config.config_sensepi,sizeof(local_sensepi_config_t));
    led_sense_conf(&config);
    working_mode = local_sensepi_config_t.trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config_t.pir_conf.threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = (local_sensepi_config_t.pir_conf.intr_trig_timer)*100;
            intr_trig_time_in = LFCLK_TICKS_MS(intr_trig_time_in);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_cam_trigger->config_sensepi->pir_conf.amplification);
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = (local_sensepi_config_t.timer_conf.timer_interval) * 100;
            break;
        }
        case PIR_AND_TIMER:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config_t.pir_conf.threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = (local_sensepi_config_t.pir_conf.intr_trig_timer)*100;
            intr_trig_time_in = LFCLK_TICKS_MS(intr_trig_time_in);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_cam_trigger->config_sensepi->pir_conf.amplification);
            timer_interval_in = (local_sensepi_config_t.timer_conf.timer_interval) * 100;
            break;
        }
    }

    out_gen_init(NUM_PIN_OUT, config.signal_out_pin_array, (bool *) out_gen_end_all_on);
    out_gen_stop((bool *) out_gen_end_all_on);
}

void sensepi_cam_trigger_update(sensepi_config_t * update_config)
{
    uint32_t working_mode = 0;
    log_printf("SensePi Cam Trigger Update\n");
    memcpy(config.config_sensepi, update_config, sizeof(sensepi_config_t));
    working_mode = config.config_sensepi->trig_conf;
    switch((trigger_conf_t)working_mode)
    {
        case PIR_ONLY:
        {
            config_pir.threshold = ((uint32_t) config.config_sensepi->pir_conf.threshold)
                    *PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(config.config_sensepi->pir_conf.amplification);
            intr_trig_time_in = ((uint32_t)config.config_sensepi->pir_conf.intr_trig_timer)*100;
            intr_trig_time_in = LFCLK_TICKS_MS(intr_trig_time_in);
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = (config.config_sensepi->timer_conf.timer_interval) * 100;
            break;
        }
        case PIR_AND_TIMER:
        {
            config_pir.threshold = ((uint32_t) config.config_sensepi->pir_conf.threshold)
                    *PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(config.config_sensepi->pir_conf.amplification);
            intr_trig_time_in = ((uint32_t)config.config_sensepi->pir_conf.intr_trig_timer)*100;
            intr_trig_time_in = LFCLK_TICKS_MS(intr_trig_time_in);
            timer_interval_in = (config.config_sensepi->timer_conf.timer_interval) * 100;
            break;
        }
    }
}

void sensepi_cam_trigger_add_tick(uint32_t interval)
{
    log_printf("SensePi Add ticks : %d\n", interval);
    //TODO Take care of this if
    //NOTE if this if() removed then state switching does not happen
//    if(interval > 1000)
//    {
        start_sense();
//    }
}

/*Data_Process module*/

void pattern_out_done_handler(uint32_t out_gen_state)
{
    log_printf("Out Gen Done\n");
    log_printf("OUT_GEN_STATE : %02x\n", out_gen_state);

    switch((cam_trig_state_t)out_gen_state)
    {
        case IDLE:
        {
            if((bool)(config.config_sensepi->trig_conf == TIMER_ONLY) == false)
            {
                pir_enable();
                if(pir_oper_flag == true)
                {
                    pir_oper_flag = false;
                }
            }
            break;
        }
        case VIDEO_PIR_ON:
        {
            video_on_flag = true;
            pir_enable();
            out_gen_config_t video_end_out_gen_config = 
            {
                .num_transitions = 2,
                .out_gen_done_handler = pattern_out_done_handler,
                .out_gen_state = VIDEO_END,
                .transitions_durations =
                    {time_remain, VIDEO_CONTROL_PULSE},
                .next_out = 
                    {{1,0,1},
            {1,1,1}},
            };
            out_gen_start(&video_end_out_gen_config);
            debug_print_bool_array(video_end_out_gen_config.next_out,
                    "Pattern done video start");
            break;
        }
        case VIDEO_END:
        {
            video_on_flag = 0;
            pir_enable();
            break;
        }
        case TIMER_IDLE:
        {
            break;
        }
    }
}

sensepi_config_t * sensepi_cam_trigger_get_sensepi_config_t()
{
    log_printf("SensePi_PIR_get_config\n");
    return (config.config_sensepi);
}

uint32_t data_process_get_video_extention (void)
{
    return (uint32_t)config.config_sensepi->pir_conf.intr_trig_timer * 100;
}

void single_shot_mode()
{
    log_printf("SINGLE_SHOT_MODE\n");
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    uint32_t local_out_gen_state;
    if(pir_oper_flag == true)
    {
        time_remain = intr_trig_time_in - (SINGLE_SHOT_DURATION); 
        local_out_gen_state = IDLE;
    }
    else
    {
        local_out_gen_state = TIMER_IDLE;
        time_remain = LFCLK_TICKS_MS(1);
    }

    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = out_gen_done_ptr,
        .out_gen_state = local_out_gen_state,
        .transitions_durations = { SINGLE_SHOT_DURATION, time_remain },
        .next_out = {{0, 1, 1},
                     {0, 1, 1}},
    };

    debug_print_bool_array(out_gen_config.next_out, "single shot");
    out_gen_start(&out_gen_config);  
}

void multi_shot_mode(uint32_t burst_duration, uint32_t burst_num)
{
    uint32_t local_out_gen_state;
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS * burst_num;
    log_printf("MULTISHOT MODE\n");
    //Time for trigger pulse and time till next trigger for each burst
    uint32_t repeat_delay_array[SINGLE_SHOT_TRANSITIONS] = {SINGLE_SHOT_DURATION,
            LFCLK_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION};
    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = out_gen_done_ptr,
//        .out_gen_state = IDLE,
    };

    for(uint32_t i = 0; i< burst_num; i++)
    {
        memcpy(out_gen_config.transitions_durations + i*SINGLE_SHOT_TRANSITIONS,
                repeat_delay_array, SINGLE_SHOT_TRANSITIONS*sizeof(uint32_t));

        for(uint32_t j = 0; j < NUM_PIN_OUT; j++)
        {
            memcpy(*(out_gen_config.next_out +j),*(multishot_generic + j),
                    burst_num* SINGLE_SHOT_TRANSITIONS*sizeof(bool));
            //An extra '1' at the end for the remaining of the inter-trigger time
            out_gen_config.next_out[j][burst_num* SINGLE_SHOT_TRANSITIONS] = 1;
        }
    }

    if(pir_oper_flag == true)
    {
        time_remain = intr_trig_time_in - SINGLE_SHOT_DURATION*burst_num -
            (LFCLK_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION)*(burst_num - 1);  
        local_out_gen_state = IDLE;
   }
    else
    {
        time_remain = LFCLK_TICKS_MS(1);
        local_out_gen_state = TIMER_IDLE;
    }
    out_gen_config.out_gen_state = local_out_gen_state;
    // Last interval for the '1' signal till 'time till next trigger' elapses
    out_gen_config.transitions_durations[number_of_transition-1] = time_remain;
            

    debug_print_bool_array(out_gen_config.next_out, "multi shot");

#if DEBUG_PRINT
    for(uint32_t i = 0; i<number_of_transition; i++)
    {
        log_printf(": %d", out_gen_config.transitions_durations[i]);
    }
#endif
    out_gen_start(&out_gen_config);
}

void bulb_mode(uint32_t bulb_time)
{
    log_printf("BULB_MODE\n");
    uint32_t local_out_gen_state;
    uint32_t number_of_transition = BULB_SHOT_TRANSITIONS;
    uint32_t bulb_time_ticks = LFCLK_TICKS_MS((bulb_time*100));

    if(pir_oper_flag == true)
    {
        time_remain = intr_trig_time_in - bulb_time_ticks ;
        local_out_gen_state = IDLE;
    }
    else
    {
        time_remain = LFCLK_TICKS_MS(1);
        local_out_gen_state = TIMER_IDLE;
    }

    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = out_gen_done_ptr,
        .out_gen_state = local_out_gen_state,
        .transitions_durations = 
            {bulb_time_ticks - BULB_TRIGGER_PULSE, BULB_TRIGGER_PULSE
              , time_remain},
        .next_out = { {0, 0, 1, 1},
                      {1, 0, 1, 1} },
    };

    debug_print_bool_array(out_gen_config.next_out, "bulb mode");

    out_gen_start(&out_gen_config);
}

void video_mode(uint32_t video_time)
{
    log_printf("VIDEO_MODE\n");

    //With PIR
    if(false == (config.config_sensepi->trig_conf == TIMER_ONLY))
    {
        video_ext_time = data_process_get_video_extention();
        video_ext_time = LFCLK_TICKS_MS(video_ext_time);
        uint32_t video_pir_disable_len = LFCLK_TICKS_MS((video_time * 100)) -
                video_ext_time;
        out_gen_config_t out_gen_config =
        {
            .num_transitions = VIDEO_TRANSITION,
            .out_gen_done_handler = out_gen_done_ptr,
            .out_gen_state = VIDEO_PIR_ON,
            .transitions_durations =
            {VIDEO_CONTROL_PULSE, (video_pir_disable_len)},
            .next_out = {{0,1,1},
                         {1,1,1}},
        };

        time_remain = video_ext_time;
        debug_print_bool_array(out_gen_config.next_out, "video PIR mode");

        out_gen_start(&out_gen_config);
    }
    //Only Timer
    else
    {
        uint32_t time_left = timer_interval_in - ((video_time * 100) - 500);
        log_printf("time remain: %d\n",time_left);
        time_left = LFCLK_TICKS_MS(time_left);

        out_gen_config_t out_gen_config =
        {
            .num_transitions = VIDEO_TRANSITION * 2,
            .out_gen_done_handler = out_gen_done_ptr,
            .out_gen_state = TIMER_IDLE,
            .transitions_durations = {VIDEO_CONTROL_PULSE,
                LFCLK_TICKS_MS(video_time*100), VIDEO_CONTROL_PULSE, time_left},
            .next_out = {{0, 1, 0, 1, 1},
                         {1, 1, 1, 1, 1}},
        };

        debug_print_bool_array(out_gen_config.next_out, "video timer mode");

        out_gen_start(&out_gen_config);
    }
}

void focus_mode()
{
    uint32_t local_out_gen_state;
    log_printf("FOCUS_MODE\n");
    uint32_t number_of_transition = FOCUS_TRANSITIONS;

    if(pir_oper_flag == true)
    {
        time_remain =  intr_trig_time_in - SINGLE_SHOT_DURATION ;
        local_out_gen_state = IDLE;
    }
    else
    {
        time_remain = LFCLK_TICKS_MS(1);
        local_out_gen_state = TIMER_IDLE;
    }

    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .out_gen_done_handler = out_gen_done_ptr,
        .out_gen_state = local_out_gen_state,
        .transitions_durations =
        {SINGLE_SHOT_DURATION ,time_remain},
        .next_out = { {0, 1, 1},
                      {1, 1, 1} },
    };
    
    debug_print_bool_array(out_gen_config.next_out, "focus mode");
    
    out_gen_start(&out_gen_config);

}

void data_process_pattern_gen(bool data_process_mode)
{
    log_printf("Pattern_Generation\n");
    uint32_t config_mode;

//    out_gen_done_ptr = pattern_out_done_handler;
    
    if((data_process_mode_t)data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        config_mode = config.config_sensepi->pir_conf.mode;
        out_gen_done_ptr = pattern_out_done_handler;
    }
    else
    {
        config_mode = config.config_sensepi->timer_conf.mode;
        out_gen_done_ptr = NULL;
    }
//    log_printf("Out_Gen_Done_Ptr = %x\n",out_gen_done_ptr);
    out_gen_stop((bool *) out_gen_end_all_on);
    uint32_t mode = ((config_mode & MODE_MSK) 
            >> (POS_OF_MODE * SIZE_OF_BYTE));
    uint32_t input1 = ((config_mode & INPUT1_MSK) 
            >> (POS_OF_INPUT1 * SIZE_OF_BYTE));
    uint32_t input2 = ((config_mode & INPUT2_MSK) 
            >> (POS_OF_INPUT2 * SIZE_OF_BYTE));
    uint32_t single_input = config_mode>> (POS_OF_INPUT1 * SIZE_OF_BYTE);
#if DEBUG_PRINT
    log_printf("Mode : %02x\n", mode);
    log_printf("Input 1 : %06d\n", input1);
    log_printf("input 2 : %02x\n", input2);
#endif
    switch((operational_mode_t)mode)
    {
        case MODE_SINGLE_SHOT:
        {
            single_shot_mode();
            break;
        }
        case MODE_MULTISHOT:
        {
            multi_shot_mode(input1, input2);
            break;
        }
        case MODE_BULB :
        {
            //Using both input 1 and input 2
            bulb_mode(single_input);
            break;
        }
        case MODE_VIDEO :
        {
            //Using both input 1 and input 2
            video_mode(single_input);
            break;
        }
        case MODE_FOCUS :
        {
            focus_mode();
            break;
        }
    }   
      
}
