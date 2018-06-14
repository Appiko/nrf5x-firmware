/* 
 * File:   sensepi_pir.c
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


#include "sensepi_pir.h"

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

/*Sensepi_PIR module MACROS*/
#define LED_WAIT_TIME_MS 301
#define PIR_SENSE_INTERVAL_MS 50
#define PIR_SENSE_THRESHOLD 600
#define PIR_THRESHOLD_MULTIPLY_FACTOR 8
#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32
#define SENSEPI_PIR_SLOW_TICK_INTERVAL_MS 300000

/*Data_Process module MACROS*/
#define PIR_DATA_PROCESS_MODE false

#define TIMER_DATA_PROCESS_MODE true

#define SIZE_OF_BYTE 8
#define POS_OF_MODE 0
#define POS_OF_INPUT1 1
#define POS_OF_INPUT2 3
#define MODE_MSK 0x000000FF
#define INPUT1_MSK 0x00FFFF00
#define INPUT2_MSK 0xFF000000

#define MODE_SINGLE_SHOT    0x00
#define MODE_MULTISHOT      0x01
#define MODE_BULB           0x02
#define MODE_VIDEO          0x03
#define MODE_FOCUS          0x04

#define NUM_PIN_OUT 2
#define FOCUS_TRIGGER_TIME_DIFF LFCLK_TICKS_977(10)
#define SINGLE_SHOT_TRANSITIONS 4
#define SINGLE_SHOT_DURATION LFCLK_TICKS_977(250)
#define VIDEO_TRANSITION 2
#define VIDEO_CONTROL_PULSE LFCLK_TICKS_977(100)
#define VIDEO_EXTENTION_TRANSITION 7
#define FOCUS_TRANSITIONS 2


//Not a pointer, but a variable that gets the config copied to
/*SensePi_PIR module variables.*/
static sensepi_pir_config_t config;
static bool pir_sense_flag = 1;
static pir_sense_cfg config_pir;
static uint32_t intr_trig_time_in = 0;
static uint32_t intr_trig_time_count = 0;
static uint32_t timer_interval_in = 0;
static uint32_t working_mode = 0;
static uint32_t video_pir_disable_len = 0;
static bool video_on_flag = 1;
static uint32_t video_ext_time = 0;
static bool sensepi_pir_start_flag = 0;
//static bool timer_done_flag = 0;
sensepi_config current_config;

/*Data_Process module variables*/
static bool one_click_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS + 1] = {{0,0,0,1},
    {1,0,1,1,}};
static sensepi_config config_sensepi;
static uint32_t delay_array[OUT_GEN_MAX_TRANSITIONS] = {};
static bool out_pattern[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS] = {};
static uint32_t number_of_transition = 0;
static uint32_t config_mode;
static uint32_t pin_outs[NUM_PIN_OUT];
static uint32_t time_remain = 0;
static uint32_t total_delay = 0;
static uint32_t video_current_time = 0;

/**Enum with all the substate for SENSING state*/
typedef enum
{
    PIR_ENABLED,
    PIR_DISABLED,
}sensepi_pir_substate;

static sensepi_pir_substate pir_substate = PIR_ENABLED;
/**
 * @brief Function to enable PIR sensing
 */
void pir_enable();

/**
 * @brief Function to disable PIR sensing
 */
void pir_disable();

/**
 * @brief Function which will handle PIR operations.
 * @param interval Interval value sent from main application
 */
void pir_operation(uint32_t interval);

/**IRQ Handler for PIR interrupt*/
void pir_handler(int32_t adc_val);

void timer_handler(void);

/**
 * @brief Function to enable LED sensing with proper parameters.
 * @param led_conf
 */
void led_sense_conf(sensepi_pir_config_t * led_conf);

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
 * @brief Function to Check if operational mode is Video. And if yes check if
 * PIR is needed to switch on yet or not.
 * @param interval Interval since last tick
 * @return 1 if:\
 * operational mode is not video or specified time has passed.
 * @return 0 if:\
 * operational mode is video and specified time has not passed. 
 */
bool check_video_len(uint32_t interval);

/*Data_Process module functions*/
    
/**
 * @brief Function which is to be called to generate and send output pattern
 * @param data_process_mode boolean value to select from which configuration
 * mode has to be selected
 */
void data_process_pattern_gen(bool data_process_mode);
/**
 * @brief Function to store configuration received from mobile app.
 * @param config Configuration for which local copy has to be created.
 */
void data_process_config(sensepi_config *local_config, uint32_t * out_pin_array);

/**
 * @brief Function which is to be called when Sensepi_PIR module is being \
 * terminated
 */
void data_process_stop();
/**
 * @brief Function to add extention to video
 * @param time_remaining How much time is remaining out of original video len.
 * @note This function will come in play only when PIR is being used and mode of
 * operation is video mode.
 */
void data_process_add_video_ext(uint32_t time_remaining);
/**
 * @brief Function to get Extention value.
 * @return Extention time
 */
uint32_t data_process_get_video_extention(void);

void single_shot_mode();
void multi_shot_mode(uint32_t input1, uint32_t input2);
void bulb_mode(uint32_t input1, uint32_t input2);
void video_mode(uint32_t input1, uint32_t input2);
void focus_mode();

void pattern_out_done_handler(out_gen_state_t out_gen_state);

///Function definations
/*SensePi_PIR module*/
void pir_enable()
{
    if(config.config_sensepi->trig_conf != TIMER_ONLY)
    {
        log_printf("PIR_Enabled\n");
        pir_sense_start(&config_pir);
        pir_sense_flag = 1;
        device_tick_switch_mode(DEVICE_TICK_SLOW);
        pir_substate = PIR_ENABLED;
    }
    return;
}

void pir_disable()
{
    log_printf("PIR_Disabled\n");
    pir_sense_stop();
    pir_substate = PIR_DISABLED;
    intr_trig_time_count = 0;
    pir_sense_flag = 0;
}

void pir_operation(uint32_t interval)
{
    log_printf("PIR_Operation\n");
    switch(pir_substate)
    {
        case PIR_ENABLED:
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time) == false)
            {
                pir_substate = PIR_DISABLED;
            }
            break;
        }
        case PIR_DISABLED :
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time) == true)
            {
                intr_trig_time_count += interval;
                if(intr_trig_time_count >= intr_trig_time_in)
                {
                    pir_enable();
                }
            }
            break;
        }
    }

}

void pir_handler(int32_t adc_val)
{
    log_printf("Sensed %d\n", adc_val);
//    device_tick_switch_mode(DEVICE_TICK_FAST);
    data_process_pattern_gen(PIR_DATA_PROCESS_MODE);
    if(pir_sense_flag == 1)
    {
        pir_disable();
    }
    if((config.config_sensepi->pir_conf->mode && MODE_MSK) >>
            (POS_OF_MODE * SIZE_OF_BYTE) == MODE_VIDEO && video_on_flag == true)
    {
        video_ext_time = data_process_get_video_extention();
        time_remain -= (video_current_time);
        bool both_on[] = {1, 1};
        out_gen_stop(both_on);
        out_gen_config_t video_end_out_gen_config = 
        {
            .num_transitions = 0,
            .transitions_durations[] = {video_ext_time},
            .next_out[] = {{1},
            {1}},
            .out_gen_done_handler = pattern_out_done_handler,
            .out_gen_state = VIDEO_START,
        };
        out_gen_start(video_end_out_gen_config);
    }
}

void timer_handler(void)
{
    log_printf("Timer Handler\n");
    data_process_pattern_gen(TIMER_DATA_PROCESS_MODE);
//    ms_timer_stop(MS_TIMER1);
    ms_timer_start(MS_TIMER2,
        MS_REPEATED_CALL,
        LFCLK_TICKS_977(config.config_sensepi->timer_conf->timer_interval),
        timer_handler);

}

void led_sense_conf(sensepi_pir_config_t * led_conf)
{
    log_printf("Led_Sense_Conf\n");
    led_sense_init(led_conf->led_sense_out_pin,
            led_conf->led_sense_analog_in_pin,
            led_conf->led_sense_off_val);
}

bool light_check(oper_time_t oper_time_temp)
{
    log_printf("Light check\n");
    log_printf("Light intensity : %d", led_sense_get());
    static uint8_t light_sense_config = 1;
    static uint32_t light_threshold = 0;
    static bool light_check_flag = 0;
    light_sense_config = oper_time_temp.day_or_night;
    light_threshold = (uint32_t)((oper_time_temp.threshold) * LIGHT_THRESHOLD_MULTIPLY_FACTOR);
    if(light_sense_config == 1 && led_sense_get() >= light_threshold)
    {
        light_check_flag = 1;
    }
    else if(light_sense_config == 0 && led_sense_get() <= light_threshold)
    {
        light_check_flag = 1;
    }
    else
    {
        light_check_flag = 0;
        device_tick_switch_mode(DEVICE_TICK_SLOW);
    }
    return light_check_flag;

}

void sensepi_pir_start()
{
    if(sensepi_pir_start_flag == 0)
    {
        log_printf("SensePi_PIR_Start\n");
        led_sense_cfg_input(true);
        hal_nop_delay_ms(LED_WAIT_TIME_MS);
        data_process_config(config.config_sensepi, config.signal_out_pin_array);
        timer_interval_in = config.config_sensepi->timer_conf->timer_interval;
        ms_timer_start(MS_TIMER2,
            MS_REPEATED_CALL,
            LFCLK_TICKS_977(config.config_sensepi->timer_conf->timer_interval),
            timer_handler);
        sensepi_pir_start_flag = 1;
    }
}

void sensepi_pir_stop()
{
    log_printf("SensePi_PIR_Stop\n");
    led_sense_cfg_input(false);
    pir_disable();
    ms_timer_stop(MS_TIMER2);
    sensepi_pir_start_flag = 0;

}

void sensepi_pir_init(sensepi_pir_config_t * config_sensepi_pir)
{
    log_printf("SensePi_PIR_init\n");
    memcpy(&config, config_sensepi_pir, sizeof(config));
    sensepi_config local_sensepi_config;
    memcpy(&local_sensepi_config, config.config_sensepi,sizeof(local_sensepi_config));
    working_mode = local_sensepi_config.trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config.pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = local_sensepi_config.pir_conf->intr_trig_timer;
            led_sense_conf(&config);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_pir->config_sensepi->pir_conf->amplification);
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = local_sensepi_config.timer_conf->timer_interval;
            break;
        }
        case PIR_AND_TIMER:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config.pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = local_sensepi_config.pir_conf->intr_trig_timer;
            led_sense_conf(&config);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_pir->config_sensepi->pir_conf->amplification);
            timer_interval_in = local_sensepi_config.timer_conf->timer_interval;
            break;
        }
    }
    return;
}

void sensepi_pir_update(sensepi_config * update_config)
{
    log_printf("SensePi_PIR_update\n");
    working_mode = update_config->trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            config_pir.threshold = ((uint32_t) update_config->pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(update_config->pir_conf->amplification);
            intr_trig_time_in = (uint32_t)update_config->pir_conf->intr_trig_timer;
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = update_config->timer_conf->timer_interval;
            break;
        }
        case PIR_AND_TIMER:
        {
            config_pir.threshold = ((uint32_t) update_config->pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(update_config->pir_conf->amplification);
            intr_trig_time_in = (uint32_t)update_config->pir_conf->intr_trig_timer;
            timer_interval_in = update_config->timer_conf->timer_interval;
            break;
        }
    }
    data_process_config(update_config, config.signal_out_pin_array);
}

void sensepi_pir_add_tick(uint32_t interval)
{
    log_printf("SensePi Add ticks : %d", interval);
    working_mode = config.config_sensepi->trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time))
            {
                sensepi_pir_start();
            }
            else
            {
                sensepi_pir_stop();
            }
            break;
        }
        case TIMER_ONLY:
        {
            if(light_check(config.config_sensepi->timer_conf->oper_time) == true)
            {
                sensepi_pir_start();
            }
            else
            {
                sensepi_pir_stop();
            }
            break;
        }
        case PIR_AND_TIMER:
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time) || 
            light_check(config.config_sensepi->timer_conf->oper_time) == true)
            {
                sensepi_pir_start();
            }
            else
            {
                sensepi_pir_stop();
            }
            break;
        }
        
    }
}

/*Data_Process module*/

void pattern_out_done_handler(out_gen_state_t out_gen_state)
{
    switch(out_gen_state)
    {
        case IDLE:
        {
            pir_enable();
            break;
        }
        case PHOTO:
        {
            out_gen_config_t idle_out_gen_config =
            {
                .num_transitions = 1,
                .transitions_durations[OUT_GEN_MAX_TRANSITIONS] = {time_remain},
                .next_out = {{1,1},
                {1,1}},
                .out_gen_done_handler = pattern_out_done_handler,
                .out_gen_state = IDLE,
            };
            out_gen_start(idle_out_gen_config);
            break;
        }
        case VIDEO_START:
        {
            video_on_flag = 1;
            pir_enable();
            out_gen_config_t video_end_out_gen_config = 
            {
                .num_transitions = 2,
                .transitions_durations = {time_remain, VIDEO_CONTROL_PULSE,VIDEO_CONTROL_PULSE},
                .next_out = {{1,0,1},
                {1,1,1}},
                .out_gen_done_handler = pattern_out_done_handler,
                .out_gen_state = VIDEO_END,
            };
            out_gen_start(video_end_out_gen_config);
            break;
        }
        case VIDEO_END:
        {
            video_on_flag = 0;
            pir_enable();
            break;
        }
    }
}

sensepi_config * sensepi_pir_get_sensepi_config()
{
    log_printf("SensePi_PIR_get_config\n");
    return (config.config_sensepi);
}

void data_process_config(sensepi_config *local_config, uint32_t *out_pin_array )
{
    log_printf("Data_Process_CONF\n");
    memcpy(&config_sensepi, local_config,sizeof(config));
    out_gen_init(NUM_PIN_OUT, out_pin_array);
    memcpy(pin_outs, out_pin_array, sizeof(pin_outs));
#if 0
    for(uint32_t pin_num = 0; pin_num < ARRAY_SIZE(pin_outs); pin_num++)
    {
        log_printf("OUT_PIN_ARRAY[%d] : %d\n", pin_num, out_pin_array[pin_num]);        
    }
    for(uint32_t pin_num = 0; pin_num < ARRAY_SIZE(pin_outs); pin_num++)
    {
        log_printf("PIN_OUT[%d] : %d\n", pin_num, pin_outs[pin_num]);        
    }
#endif
}

void single_shot_mode()
{
    log_printf("SINGLE SHOT MODE\n");
    number_of_transition = SINGLE_SHOT_TRANSITIONS;
    uint32_t local_delay_array[] = {SINGLE_SHOT_DURATION,
    FOCUS_TRIGGER_TIME_DIFF,
    SINGLE_SHOT_DURATION, 
    FOCUS_TRIGGER_TIME_DIFF,
    SINGLE_SHOT_DURATION};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS+1] = {{1, 0, 0, 0, 1},
        {1, 1, 0, 1, 1}};
    for(uint32_t row; row < NUM_PIN_OUT; row++)
    { 
        memcpy(&out_pattern[row][0], &local_out_pattern[row][0],
                (SINGLE_SHOT_TRANSITIONS+1));
    } 
    total_delay = 0;
    for(uint32_t arr_c = 0; arr_c < ARRAY_SIZE(delay_array); arr_c++)
    {
        total_delay += delay_array[arr_c];
    }
    time_remain = LFCLK_TICKS_977(intr_trig_time_in) + (1<<24) - (total_delay);
    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .transitions_durations = delay_array,
        .next_out = out_pattern,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = PHOTO,
    };
    out_gen_start(out_gen_config);
    
}
void multi_shot_mode(uint32_t input1, uint32_t input2)
{
    number_of_transition = SINGLE_SHOT_TRANSITIONS * input2;
    uint32_t repeat_delay_array[] = {FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, LFCLK_TICKS_MS(input1)};
    delay_array[0] = SINGLE_SHOT_DURATION;
    out_pattern[0][0] = 1;
    out_pattern[1][0] = 1;
    for(uint32_t loop = 1; loop <= number_of_transition; loop = loop+SINGLE_SHOT_TRANSITIONS)
    {
        memcpy(&delay_array[loop], repeat_delay_array, sizeof(repeat_delay_array));
        memcpy(&(out_pattern[0][loop]), &one_click_pattern[0][loop],SINGLE_SHOT_TRANSITIONS);
        memcpy(&(out_pattern[1][loop]), &one_click_pattern[1][loop],SINGLE_SHOT_TRANSITIONS);

    }

    for(uint32_t row = 0; row < NUM_PIN_OUT; row++)
    {
        log_printf("ROW[%d]: \n", row);
        for(uint32_t col = 0; col < ARRAY_SIZE(delay_array); col++)
        {
            log_printf("    COL[%d] : %d,  %d\n", col, out_pattern[row][col], delay_array[col]);
        }
        log_printf("\n\n");
    }
    time_remain = LFCLK_TICKS_977(intr_trig_time_in) + (1<<24) - (total_delay);
    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .transitions_durations = delay_array,
        .next_out = out_pattern,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = PHOTO,
    };
    out_gen_start(out_gen_config);

}

void bulb_mode(uint32_t input1, uint32_t input2)
{
    number_of_transition = SINGLE_SHOT_TRANSITIONS;
    uint32_t local_delay_array[SINGLE_SHOT_TRANSITIONS + 1] = {SINGLE_SHOT_DURATION, LFCLK_TICKS_977((input1+input2) - 4), LFCLK_TICKS_977(4), FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS + 1] = {
        {1, 0, 0, 0, 1},
        {1, 1, 0, 1, 1}};
    memcpy(out_pattern, local_out_pattern, sizeof(local_out_pattern));
    time_remain = LFCLK_TICKS_977(intr_trig_time_in) + (1<<24) - (total_delay);
    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .transitions_durations = delay_array,
        .next_out = out_pattern,
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = PHOTO,
    };
    out_gen_start(out_gen_config);

}

void video_mode(uint32_t input1, uint32_t input2)
{
    video_ext_time = data_process_get_video_extention();
    video_pir_disable_len = input1 - video_ext_time;
    number_of_transition = VIDEO_TRANSITION;
    uint32_t local_delay_array[VIDEO_TRANSITION + 1] = 
    {FOCUS_TRIGGER_TIME_DIFF, VIDEO_CONTROL_PULSE, video_pir_disable_len};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][VIDEO_TRANSITION + 1] = {{1, 0, 1},
        {1, 1, 1}};
    for(uint32_t row; row < NUM_PIN_OUT; row++)
    { 
        memcpy(&out_pattern[row][0], &local_out_pattern[row][0], (VIDEO_TRANSITION+1));
    }
        out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .transitions_durations = delay_array,
        .next_out[OUT_GEN_MAX_TRANSITIONS] = out_pattern[OUT_GEN_MAX_TRANSITIONS],
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = VIDEO_START,
    };
    out_gen_start(out_gen_config);
    
}

void focus_mode()
{
    number_of_transition = 2;
    uint32_t local_delay_array[3] = {SINGLE_SHOT_DURATION, SINGLE_SHOT_DURATION * 20, SINGLE_SHOT_DURATION};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][FOCUS_TRANSITIONS + 1] = {
        {1, 0, 1},
        {1, 0, 1}
    };
    for(uint32_t row; row < NUM_PIN_OUT; row++)
    { 
        memcpy(&out_pattern[row][0], &local_out_pattern[row][0], 3);
    }
    time_remain = LFCLK_TICKS_977(intr_trig_time_in) + (1<<24) - (total_delay);
    out_gen_config_t out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .transitions_durations = delay_array,
        .next_out[OUT_GEN_MAX_TRANSITIONS] = out_pattern[OUT_GEN_MAX_TRANSITIONS],
        .out_gen_done_handler = pattern_out_done_handler,
        .out_gen_state = PHOTO,
    };
    out_gen_start(out_gen_config);

}
/**To Generate pattern which is to be sent over pins.*/
void data_process_pattern_gen(bool data_process_mode)
{
    log_printf("Pattern_Gen : %d \n", data_process_mode);
    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        config_mode = config_sensepi.pir_conf->mode;
    }
    else
    {
        config_mode = config_sensepi.timer_conf->mode;
    }
    for(uint32_t pin_num = 0; pin_num < ARRAY_SIZE(pin_outs); pin_num++)
    {
//        log_printf("OUTPUT_SET : %d\n",pin_outs[pin_num]);
        hal_gpio_cfg_output(pin_outs[pin_num], 1);
    }
    uint32_t mode = (config_mode & MODE_MSK) >> (POS_OF_MODE * SIZE_OF_BYTE);
    uint32_t input1 = (config_mode & INPUT1_MSK) >> (POS_OF_INPUT1 * SIZE_OF_BYTE);
    uint32_t input2 = (config_mode & INPUT2_MSK) >> (POS_OF_INPUT2 * SIZE_OF_BYTE);
#if 0
    log_printf("Mode : %02x\n", mode);
    log_printf("Input 1 : %04x\n", input1);
    log_printf("input 2 : %02x\n", input2);
#endif    
    switch(mode)
    {
        case MODE_SINGLE_SHOT:
        {
//            single_shot
            single_shot_mode();
            break;
        }
        
        case MODE_MULTISHOT:
        {
            log_printf("MULTISHOT MODE\n");
//            multishot(input1, input2);

//            input1 = 50;
//            input2 = 4;
            multi_shot_mode(input1, input2);
            break;
        }
        case MODE_BULB :
        {
//            bulb(input1, input2)
#if 0
            input1 = 1500;
            input2 = 10;
#endif
            bulb_mode(input1,input2);
            break;
        }
        case MODE_VIDEO :
        {
//            video(input1, input2)
//            input1 = 60;
//            input2 = 10;
            video_mode(input1, input2);
            break;
        }
        case MODE_FOCUS :
        {
            focus_mode();
            break;
        }
  
    }
#if 0
    log_printf("Number of Transition : %d\n", number_of_transition);
    log_printf("Delay Array : \n");
    for(uint32_t arr_p = 0; arr_p<ARRAY_SIZE(delay_array); arr_p++)
    {
        log_printf("%d ", delay_array[arr_p]);
    }
    log_printf("\n");
    log_printf("Out Pattern :\n");
    for(uint32_t row = 0; row< NUM_PIN_OUT; row++)
    {
        for(uint32_t arr_p = 0; arr_p<ARRAY_SIZE(out_pattern[0]); arr_p++)
        {
            log_printf("%x ", out_pattern[row][arr_p]);
        }
        log_printf("\n");
    }
    log_printf("\n");
    if(mode != MODE_VIDEO)
    {
        out_gen_start(number_of_transition, delay_array, out_pattern);    
    }
    else
    {
        out_gen_stop(true);
        
    }
#endif
//    out_gen_start(number_of_transition, delay_array, out_pattern, mode_patt_out_done);    
    return;        
}

void data_process_stop()
{
    log_printf("Data Process Stop\n");
    hal_gpio_cfg_output(JACK_FOCUS_PIN, 1);
    hal_gpio_cfg_output(JACK_TRIGGER_PIN, 1);
}
