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
#define PIR_SENSE_INTERVAL_MS 40
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
#define BULB_SHOT_TRANSITIONS 2
/** Number of transitions required for focus operation */
#define FOCUS_TRANSITIONS 2
/** Number of transitions required for Video operation triggered by Timer */
#define TIMER_VIDEO_TRANSITIONS 3
/** Ticks duration of trigger pulse to start a video */
#define VIDEO_START_PULSE MS_TIMER_TICKS_MS(250)
/** Ticks duration of trigger pulse to end a video */
#define VIDEO_END_PULSE MS_TIMER_TICKS_MS(300)
/** Number of maximum extensions for Video */
#define NO_OF_VIDEO_EXTN_ALLOWED 3
/** Duration for which PIR will be active for video extension */
#define VIDEO_PIR_ON 2000

/** Address of first memory location of last byte */
#define LAST_APP_PAGE_ADDR 0x27000

/***/
#define LAST_CONFIG_ADDR 0x27FE8

/***/
#define COMP_RESET_VALUE 0xFFFFFFFF


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
    VIDEO_IDLE,
    MAX_STATES,
}
cam_trig_state_t;
/** Renaming to use memory location for out_gen_config[TIMER_IDLE] to store a
 *  config for extension as both extension and timer cannot be operated
 *  simultaneously. Renaming is done to avoid confusion.  */
#define VIDEO_ETXN_CONFIG TIMER_IDLE

/** Copy of configuration to be shared across module */
static sensepi_cam_trigger_init_config_t config;
/** PIR configuration which will be used to enable PIR */
static pir_sense_cfg config_pir;
/** Array of out_gen_config_t to store pre-calculated configs */
static out_gen_config_t out_gen_config[MAX_STATES];
/** Video Extension time in ticks */
static uint32_t video_extn_ticks;
/** Flag to keep status of PIR's expected state */
static bool pir_on_flag = false;
/** The amount of time since starting the SENSING state */
static uint32_t sense_count;
/** Boolen to indicate if PIR sensing feedback is required for user  */
static bool sense_feedback = false;
/** Flag to check if light sensing is required or not */
static bool is_light_sense_on = false;
/** Array which is to be passed while stopping out_gen module */
static const bool out_gen_end_all_on[OUT_GEN_MAX_NUM_OUT] = {1,1,1,1};
/** Array which is partially copied while generating out_gen_config for multi-shot */
static const bool multishot_generic[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS] =
    {
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
            {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}
    };

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
 * @brief PIR handler for Video mode
 * @param adc_val
 */
void pir_handler_video(int32_t adc_val);
/**
 * @brief Function to update the out_gen_configs for PIR.
 */
void out_gen_config_updater_pir(void);

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
void out_gen_config_updater_timer(void);

/*out_pattern_gen related functions*/
/**
 * @brief Handler for out pattern generation done.
 * @param out_gen_state State from which handler is called.
 */
void out_gen_done_handler(uint32_t out_gen_state);

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
void out_gen_config_single_shot(cam_trig_state_t data_process_mode);
/**
 * @brief Function to click multiple shots per trigger
 * @param data_process_mode mode for which we have to generate the out_gen_config
 * @param burst_duration Time duration between two shots
 * @param burst_num Number of shots per trigger
 */
void out_gen_config_multi_shot(cam_trig_state_t data_process_mode, 
                     uint32_t burst_duration, uint32_t burst_num);
/**
 * @brief Function to take photo in bulb mode.
 * @param data_process_mode mode for which we have to generate the out_gen_config
 * @param bulb_time Time duration for half press signal
 */
void out_gen_config_bulb_expo(cam_trig_state_t data_process_mode, uint32_t bulb_time);
/**
 * @brief Function to do focus only
 * @param data_process_mode mode for which we have to generate the out_gen_config
 */
void out_gen_config_focus_only(cam_trig_state_t data_process_mode);
/**
 * @brief Dummy mode which will do nothing i.e keep the signals high
 * @param data_process_mode mode for which we have to generate the out_gen_config
 */
void out_gen_config_none(cam_trig_state_t data_process_mode);
//Suggest some good name for this function.
/**
 * @brief Function to create out_gen_config for the video length for which
 * PIR will be enabled and if no motion is detected during this time video will
 * end
 */
void out_gen_config_part_video_end();
/**
 * @brief Function to generate out_gen_config required for extension length
 * @param extn_len Extension length provided by application in sec.
 */
void out_gen_config_part_video_extn(uint32_t extn_len);
/**
 * @brief Function to generate out_gen_config required to start a video after PIR
 * triggering with length=video_len-@ref VIDEO_PIR_ON.
 * @param video_len Length of the video without any extensions
 */
void out_gen_config_part_video_start(uint32_t video_len);
/**
 * @brief Function to record video of fix length on Timer Triggering
 * @param data_process_mode mode for which we have to generate the out_gen_config
 * @param video_len Video Length in sec.
 */
void out_gen_config_full_video(cam_trig_state_t data_process_mode, uint32_t video_len);

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

uint32_t mem_get_config_page(void)
{
    uint32_t *p_mem_loc;
    p_mem_loc = (uint32_t *) LAST_APP_PAGE_ADDR;
    uint32_t loc_no = 0;
    while(p_mem_loc <= (uint32_t *)LAST_CONFIG_ADDR)
    {
        log_printf("Location number : %d\n", ++loc_no);
        log_printf("Addr being compared : %x\n", p_mem_loc);
        if(*p_mem_loc == COMP_RESET_VALUE)
        {
            //write sensepi_config_t
            log_printf("Addr of blank location : %x\n", p_mem_loc);
            return (uint32_t)p_mem_loc;
        }
        p_mem_loc += 5;
        hal_nop_delay_us(700);
    }
    return COMP_RESET_VALUE;
}


void aapend_config_page(sensepi_config_t* new_config)
{
    uint32_t * p_mem_loc = (uint32_t *) mem_get_config_page();
    uint32_t * p_config_cast = (uint32_t *) new_config;
    if(p_mem_loc == (uint32_t *)COMP_RESET_VALUE)
    {
        log_printf("Reset Module will come here..!!\n");
    }
    else
    {
        log_printf("Blank Add : %x\n", p_mem_loc);
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        for(uint32_t i =0 ; i < 5; i++)
        {
            *p_mem_loc++ = *p_config_cast++;
        }
    	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
    }
    
}

sensepi_config_t* get_last_config(void)
{
    log_printf("%s\n", __func__);
//    uint32_t * temp_config;
    uint32_t *p_mem_loc = (uint32_t*)mem_get_config_page();
    if(p_mem_loc != (uint32_t*)LAST_APP_PAGE_ADDR && 
       p_mem_loc != (uint32_t*)LAST_CONFIG_ADDR)
    {
        p_mem_loc -= 5;
    }
    for(uint32_t i=0; i< 5; i++)
    {
//        *temp_config = *p_mem_loc;
        log_printf("%08x  ", *p_mem_loc++);
    }
    log_printf("\n");
    p_mem_loc -= 5;


//    memcpy(&temp_config,p_mem_loc,sizeof(sensepi_config_t));
    return (sensepi_config_t*) p_mem_loc;
}

void clear_config_page()
{
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);

    NRF_NVMC->ERASEPAGE = (uint32_t)LAST_APP_PAGE_ADDR;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);

}

void MemoryManagement_Handler(void)
{
    log_printf("MemoryManagement_Handler\n");
}

void HardFault_Handler(void)
{
    log_printf("%s\n", __func__);
}
void pir_set_state(bool state)
{
    log_printf("%s %x\n",__func__, state);
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
    pir_set_state(false);
    if(out_gen_is_on() == false)
    {
        if(sense_feedback == true)
        {
            led_ui_single_start(LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
        }
        out_gen_start(&out_gen_config[PIR_IDLE]);
    }
}

void pir_handler_video(int32_t adc_val)
{
    log_printf("%s", __func__);
    log_printf(" %d\n", adc_val);
    pir_set_state(false);
    static uint32_t no_of_extn_remain;
    if(out_gen_is_on() == false)
    {
        if(sense_feedback == true)
        {
            led_ui_single_start(LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
        }
        no_of_extn_remain = NO_OF_VIDEO_EXTN_ALLOWED;
        out_gen_start(&out_gen_config[PIR_IDLE]);
    }
    else if(no_of_extn_remain > 0)
    {
        uint32_t ticks_done = out_gen_get_ticks ();
        if(ticks_done < video_extn_ticks)
        {
            out_gen_config[VIDEO_ETXN_CONFIG].transitions_durations[0] = 
                (video_extn_ticks  - ticks_done);
        }
        else
        {
            out_gen_config[VIDEO_ETXN_CONFIG].transitions_durations[0] = 
                video_extn_ticks;
        }
        out_gen_start(&out_gen_config[VIDEO_ETXN_CONFIG]);
        no_of_extn_remain--;
    }
}

void out_gen_config_updater_pir()
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
            out_gen_config_single_shot(PIR_IDLE);
            break;
        }
        case MODE_MULTISHOT:
        {
            out_gen_config_multi_shot(PIR_IDLE, input1, input2);
            break;
        }
        case MODE_BULB :
        {
            //Using both input 1 and input 2
            out_gen_config_bulb_expo(PIR_IDLE, single_input);
            break;
        }
        case MODE_VIDEO :
        {
            if(input2 == 0)
            {
                out_gen_config_full_video (PIR_IDLE, input1);
            }
            else
            {
                out_gen_config_part_video_start(input1);
                out_gen_config_part_video_extn(input2);
                out_gen_config_part_video_end(input2);
            }
            break;
        }
        case MODE_FOCUS :
        {
            out_gen_config_focus_only(PIR_IDLE);
            break;
        }
        case MODE_NONE:
        {
            out_gen_config_none(PIR_IDLE);
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
        out_gen_start(&out_gen_config[TIMER_IDLE]);
    }
}

void out_gen_config_updater_timer(void)
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
            out_gen_config_single_shot(TIMER_IDLE);
            break;
        }
        case MODE_MULTISHOT:
        {
            out_gen_config_multi_shot(TIMER_IDLE, input1, input2);
            break;
        }
        case MODE_BULB :
        {
            //Using both input 1 and input 2
            out_gen_config_bulb_expo(TIMER_IDLE, single_input);
            break;
        }
        case MODE_VIDEO :
        {
            out_gen_config_full_video(TIMER_IDLE,input1);
            break;
        }
        case MODE_FOCUS :
        {
            out_gen_config_focus_only(TIMER_IDLE);
            break;
        }
        case MODE_NONE:
        {
            out_gen_config_none(PIR_IDLE);
        }
    }   
}

void out_gen_done_handler (uint32_t out_gen_state)
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
            if(config.config_sensepi->trig_conf == PIR_AND_TIMER)
            {
                pir_set_state(pir_on_flag);
            }
            break;
        }
        case VIDEO_IDLE :
        {
            out_gen_start(&out_gen_config[VIDEO_IDLE]);
            pir_set_state(pir_on_flag);
            break;
        }
        case MAX_STATES : 
        {
            log_printf("Something went seriously wrong..!!\n");
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

            light_flag = light_sense_light_check(config.config_sensepi->timer_conf.oper_time);
            timer_set_state(light_flag);
            pir_set_state(pir_on_flag);

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

void out_gen_config_single_shot(cam_trig_state_t data_process_mode)
{
    log_printf("%s\n", __func__);
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    int32_t time_remain;

    if(data_process_mode == PIR_IDLE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100) 
            - (SINGLE_SHOT_DURATION); 
        if((int32_t)time_remain <= 0)
        {
            time_remain = 0;
        }
    }
    else if(data_process_mode == TIMER_IDLE)
    {
        time_remain = 0;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = data_process_mode,
        .transitions_durations = { SINGLE_SHOT_DURATION, time_remain },
        .next_out = {{0, 1, 1},
                     {0, 1, 1}},
    };

    debug_print_bool_array(local_out_gen_config.next_out, "single shot");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void out_gen_config_multi_shot(cam_trig_state_t data_process_mode, 
                     uint32_t burst_duration, uint32_t burst_num)
{
    log_printf("%s\n", __func__);
    int32_t time_remain;
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS * burst_num;
    //Time for trigger pulse and time till next trigger for each burst
    uint32_t repeat_delay_array[SINGLE_SHOT_TRANSITIONS] = {SINGLE_SHOT_DURATION,
            MS_TIMER_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION};
    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = data_process_mode,
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

    if(data_process_mode == PIR_IDLE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - SINGLE_SHOT_DURATION*burst_num -
            (MS_TIMER_TICKS_MS(burst_duration * 100) - SINGLE_SHOT_DURATION)*(burst_num - 1);  
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
   }
    else if(data_process_mode == TIMER_IDLE)
    {
        time_remain = 1;
    }
    // Last interval for the '1' signal till 'time till next trigger' elapses
    local_out_gen_config.transitions_durations[number_of_transition-1] = time_remain;
            

    debug_print_bool_array(local_out_gen_config.next_out, "multi shot");
    memcpy(&out_gen_config[data_process_mode], &local_out_gen_config,
           sizeof(out_gen_config_t));
}

void out_gen_config_bulb_expo(cam_trig_state_t data_process_mode, uint32_t bulb_time)
{
    log_printf("%s\n", __func__);
    int32_t time_remain;
    uint32_t number_of_transition = BULB_SHOT_TRANSITIONS;
    uint32_t bulb_time_ticks = MS_TIMER_TICKS_MS((bulb_time*100));

    if(data_process_mode == PIR_IDLE)
    {
        time_remain = MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - bulb_time_ticks ;
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
    }
    else if(data_process_mode == TIMER_IDLE)
    {
        time_remain = 0;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = data_process_mode,
        .transitions_durations = 
            {bulb_time_ticks, time_remain},
        .next_out = { {0, 1, 1},
                      {0, 1, 1} },
    };

    debug_print_bool_array(local_out_gen_config.next_out, "bulb mode");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void out_gen_config_full_video(cam_trig_state_t data_process_mode, uint32_t video_len)
{
    int32_t time_remain;
    video_len = video_len*1000;
    if(data_process_mode == TIMER_IDLE)
    {
        time_remain = config.config_sensepi->timer_conf.timer_interval*100 -
            video_len - VIDEO_START_PULSE -VIDEO_END_PULSE;
    }
    video_len = (time_remain < 0) ?
        (config.config_sensepi->timer_conf.timer_interval*100 - 1000) :
        video_len;
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = TIMER_VIDEO_TRANSITIONS,
        .next_out = {{0,1,0,1},
            {1,1,1,1}},
        .transitions_durations = {VIDEO_START_PULSE,
                    MS_TIMER_TICKS_MS(video_len),
                    VIDEO_END_PULSE},
        .done_handler = out_gen_done_handler,
        .out_gen_state = data_process_mode,

    };
    debug_print_bool_array(local_out_gen_config.next_out, "Timer video mode");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));

   
}

void out_gen_config_part_video_start(uint32_t video_len)
{
    config_pir.handler = pir_handler_video;
    video_len = video_len * 1000;
    int video_len_check = video_len  - VIDEO_PIR_ON;
    video_len = (video_len_check <= 0) ? 0 : video_len_check;
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = SINGLE_SHOT_TRANSITIONS,
        .next_out = {{0,1,1}
        ,{1,1,1}},
        .transitions_durations = {VIDEO_START_PULSE, 
                                    (MS_TIMER_TICKS_MS(video_len))},
        .done_handler = out_gen_done_handler,
        .out_gen_state = VIDEO_IDLE,
    };
    debug_print_bool_array(local_out_gen_config.next_out, "PIR video mode");
    memcpy(&out_gen_config[PIR_IDLE],&local_out_gen_config,
           sizeof(out_gen_config_t));    
}

void out_gen_config_part_video_extn (uint32_t extn_len)
{
    video_extn_ticks = MS_TIMER_TICKS_MS(extn_len * 1000);
    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = 1,
        .next_out = {{1,1}, {1,1}},
        .transitions_durations = {video_extn_ticks},
        .done_handler = out_gen_done_handler,
        .out_gen_state = VIDEO_IDLE,
    };
    debug_print_bool_array(local_out_gen_config.next_out, "PIR video extension");
    memcpy(&out_gen_config[VIDEO_ETXN_CONFIG],&local_out_gen_config,
           sizeof(out_gen_config_t));    
    
}

void out_gen_config_part_video_end()
{
    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = SINGLE_SHOT_TRANSITIONS,
        .next_out = {{1,0,1},
            {1,1,1}},
        .transitions_durations = {MS_TIMER_TICKS_MS(VIDEO_PIR_ON), VIDEO_END_PULSE,},
        .done_handler = out_gen_done_handler,
        .out_gen_state = PIR_IDLE,
    };
    debug_print_bool_array(local_out_gen_config.next_out, "PIR video mode");
    memcpy(&out_gen_config[VIDEO_IDLE],&local_out_gen_config,
           sizeof(out_gen_config_t)); 
}

void out_gen_config_focus_only(cam_trig_state_t data_process_mode)
{
    log_printf("%s\n", __func__);
    int32_t time_remain;
    uint32_t number_of_transition = FOCUS_TRANSITIONS;

    if(data_process_mode == PIR_IDLE)
    {
        time_remain =  MS_TIMER_TICKS_MS(config.config_sensepi->pir_conf.intr_trig_timer * 100)
            - SINGLE_SHOT_DURATION ;
        if((int32_t)time_remain <= 0)
        {
            time_remain = 1;
        }
    }
    else if(data_process_mode == TIMER_IDLE)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = data_process_mode,
        .transitions_durations =
        {SINGLE_SHOT_DURATION ,time_remain},
        .next_out = { {0, 1, 1},
                      {1, 1, 1} },
    };
    
    debug_print_bool_array(local_out_gen_config.next_out, "focus mode");
    memcpy(&out_gen_config[data_process_mode],&local_out_gen_config,
           sizeof(out_gen_config_t));
}

void out_gen_config_none(cam_trig_state_t data_process_mode)
{
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = 1,
        .done_handler = out_gen_done_handler,
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
//    aapend_config_page(config.config_sensepi);
//    sensepi_config_t * temp_config = get_last_config ();
//    log_printf("Trig mode %d, PIR ope time %08x, PIR mode %08x, PIR amp %d, PIR thres %d, \
//         PIR int trig time %04d, Timer oper %x, Timer mode %x, timer interval %04d \n",
//        temp_config->trig_conf, temp_config->pir_conf.oper_time, temp_config->pir_conf.mode,
//        temp_config->pir_conf.amplification, temp_config->pir_conf.threshold,
//        temp_config->pir_conf.intr_trig_timer,
//        temp_config->timer_conf.oper_time, temp_config->timer_conf.mode,
//               temp_config->timer_conf.timer_interval);
//    clear_config_page();
//    mem_get_config_page();

    sense_count = 0;
    sense_feedback = true;
    oper_time_t pir_oper_time = config.config_sensepi->pir_conf.oper_time;
    oper_time_t timer_oper_time = config.config_sensepi->timer_conf.oper_time;
    bool pir_light_flag = true;
    bool timer_light_flag = true;
    if(config.config_sensepi->trig_conf != PIR_ONLY)
    {
        if((timer_oper_time.day_or_night == 1 && timer_oper_time.threshold == 0b0000000)||
        (timer_oper_time.day_or_night == 0 && timer_oper_time.threshold == 0b1111111))
        {
            timer_light_flag = false;
        }
        else
        {
            timer_light_flag = true;
        }      
    }
    else
    {
        timer_light_flag = false;
    }
    if(config.config_sensepi->trig_conf != TIMER_ONLY)
    {
        if((pir_oper_time.day_or_night == 1 && pir_oper_time.threshold == 0b0000000)||
        (pir_oper_time.day_or_night == 0 && pir_oper_time.threshold == 0b1111111))
        {
            pir_light_flag = false;
        }
        else
        {
            pir_light_flag = true;
        }      
    }
    else
    {
        pir_light_flag = false;
    }
    is_light_sense_on = pir_light_flag || timer_light_flag; 

    config_pir.threshold = ((uint32_t) config.config_sensepi->pir_conf.threshold)
            *PIR_THRESHOLD_MULTIPLY_FACTOR;
    config_pir.handler = pir_handler;

    mcp4012_set_value(config.config_sensepi->pir_conf.amplification);

    out_gen_config_updater_timer();
    out_gen_config_updater_pir();

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
    if(light_sense_count > LIGHT_SENSE_INTERVAL_TICKS && sense_feedback == false
       && is_light_sense_on == true)
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
