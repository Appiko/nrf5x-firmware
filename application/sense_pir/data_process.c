/* 
 * File:   sensepi_pir.h
 * Copyright (c) 2018 Appiko
 * Created on 25 May, 2018, 3:32 PM
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
#include "data_process.h"
#include "sensepi_ble.h"
#include "out_pattern_gen.h"
#include "nrf.h"
#include "stdint.h"
#include "nrf_util.h"
#include "log.h"
#include "hal_nop_delay.h"
#include "boards.h"
#include "stdbool.h"
#include "common_util.h"
#include "hal_gpio.h"
#include <string.h>

#define SIZE_OF_BYTE 8
#define POS_OF_MODE 0
#define POS_OF_INPUT1 1
#define POS_OF_INPUT2 3
#define MODE_MSK 0x000000FF
#define INPUT1_MSK 0x00FFFF00
#define INPUT2_MSK 0xFF000000

#define NUM_PIN_OUT 2

#define FOCUS_TRIGGER_TIME_DIFF LFCLK_TICKS_977(10)

#define SINGLE_SHOT_TRANSITIONS 4

#define SINGLE_SHOT_DURATION LFCLK_TICKS_977(250)

#define VIDEO_TRANSITION 8

#define VIDEO_STOP_PULSE LFCLK_TICKS_977(995)

#define VIDEO_EXTENTION_TRANSITION 7

#define FOCUS_TRANSITIONS 2



static bool one_click_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS + 1] = {{0,0,0,1},
    {1,0,1,1,}};

#define MODE_SINGLE_SHOT    0x00
#define MODE_MULTISHOT      0x01
#define MODE_BULB           0x02
#define MODE_VIDEO          0x03
#define MODE_FOCUS          0x04

static sensepi_config *config;

static uint32_t delay_array[OUT_GEN_MAX_TRANSITIONS] = {};
static bool out_pattern[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS] = {};
static uint32_t number_of_transition = 0;
static uint32_t config_mode;
static uint32_t pin_outs[NUM_PIN_OUT];

void single_shot_mode();
void multi_shot_mode(uint32_t input1, uint32_t input2);
void bulb_mode(uint32_t input1, uint32_t input2);
void video_mode(uint32_t input1, uint32_t input2);
void focus_mode();

void data_process_config(sensepi_config *local_config, uint32_t *out_pin_array )
{
    log_printf("Data_Process_CONF\n");
    config = local_config;
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
    uint32_t local_delay_array[] = {SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS+1] = {{1, 0, 0, 0, 1},
        {1, 1, 0, 1, 1}};
    for(uint32_t row; row < NUM_PIN_OUT; row++)
    { 
        memcpy(&out_pattern[row][0], &local_out_pattern[row][0], (SINGLE_SHOT_TRANSITIONS+1));
    } 
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

}

void video_mode(uint32_t input1, uint32_t input2)
{
    number_of_transition = VIDEO_TRANSITION;
    uint32_t local_delay_array[VIDEO_TRANSITION + 1] = {SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, LFCLK_TICKS_977(input1*1000), FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, VIDEO_STOP_PULSE, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
    memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
    bool local_out_pattern[NUM_PIN_OUT][VIDEO_TRANSITION + 1] = {{1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 1, 1, 1, 0 ,1 ,1}};
    for(uint32_t row; row < NUM_PIN_OUT; row++)
    { 
        memcpy(&out_pattern[row][0], &local_out_pattern[row][0], (VIDEO_TRANSITION+1));
    }
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
}
/**To Generate pattern which is to be sent over pins.*/
void data_process_pattern_gen(bool data_process_mode)
{
    log_printf("Pattern_Gen : %d \n", data_process_mode);
    if(data_process_mode == PIR_DATA_PROCESS_MODE)
    {
        config_mode = config->pir_conf->mode;
    }
    else
    {
        config_mode = config->timer_conf->mode;
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
#endif
    out_gen_start(number_of_transition, delay_array, out_pattern);
    return;        
}

void data_process_stop()
{
    log_printf("Data Process Stop\n");
    hal_gpio_cfg_output(JACK_FOCUS_PIN, 1);
    hal_gpio_cfg_output(JACK_TRIGGER_PIN, 1);
}
