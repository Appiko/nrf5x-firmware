/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "out_pattern_array_gen.h"
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

#define FOCUS_TRIGGER_TIME_DIFF LFCLK_TICKS_MS(10)

#define SINGLE_SHOT_TRANSITIONS 4

#define SINGLE_SHOT_DURATION LFCLK_TICKS_MS(250)

#define VIDEO_TRANSITION 8

#define VIDEO_STOP_PULSE LFCLK_TICKS_MS(995)

#define VIDEO_EXTENTION_TRANSITION 7

#define FOCUS_TRANSITIONS 2

static bool one_click_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS + 1] = {{0,0,0,1},
    {1,0,1,1,}};

#define MODE_SINGLE_SHOT    0
#define MODE_MULTISHOT      1
#define MODE_BULB           2
#define MODE_VIDEO          3
#define MODE_FOCUS          4

uint32_t out_pin_array[] = {JACK_FOCUS_PIN, JACK_TRIGGER_PIN};

/**To Generate pattern which is to be sent over pins.*/
void pattern_gen(sensepi_config *config)
{
    hal_gpio_cfg_output(JACK_FOCUS_PIN, 1);
    hal_gpio_cfg_output(JACK_TRIGGER_PIN, 1);
    uint32_t mode = config->mode & MODE_MSK >> (POS_OF_MODE * SIZE_OF_BYTE);
    uint32_t input1 = (config->mode & INPUT1_MSK) >> (POS_OF_INPUT1 * SIZE_OF_BYTE);
    uint32_t input2 = (config->mode & INPUT2_MSK) >> (POS_OF_INPUT2 * SIZE_OF_BYTE);
    uint32_t delay_array[OUT_GEN_MAX_TRANSITIONS] = {};
    bool out_pattern[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS];
    uint32_t number_of_transition = 0;
    out_gen_init(NUM_PIN_OUT, out_pin_array);
    log_printf("Mode : %02x\n", mode);
    log_printf("Input 1 : %04x\n", input1);
    log_printf("input 2 : %02x\n", input2);
    
    switch(mode)
    {
        case MODE_SINGLE_SHOT:
        {
//            single_shot
            number_of_transition = SINGLE_SHOT_TRANSITIONS;
            uint32_t local_delay_array[] = {SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
            memcpy(&delay_array[0], &local_delay_array[0], sizeof(local_delay_array));
            bool local_out_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS+1] = {{1, 0, 0, 0, 1},
                {1, 1, 0, 1, 1}};
            for(uint32_t row; row < NUM_PIN_OUT; row++)
            { 
                memcpy(&out_pattern[row][0], &local_out_pattern[row][0], (SINGLE_SHOT_TRANSITIONS+1));
            }
            for(uint32_t row = 0 ; row < NUM_PIN_OUT; row++)
            {
                log_printf("Row No: %d\n", row);
                for(uint32_t col = 0; col < number_of_transition +1; col++)
                {
                    log_printf("COL[%d] : %d, %d\n ",col,out_pattern[row][col], delay_array[col] );
                }
                log_printf("\n\n\n");
            }
            hal_nop_delay_ms(1000);
            break;
        }
        
        case MODE_MULTISHOT:
        {
//            multishot(input1, input2);
            number_of_transition = SINGLE_SHOT_TRANSITIONS * input2;
            uint32_t repeat_delay_array[] = {FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, input1};
            delay_array[0] = SINGLE_SHOT_DURATION;
            out_pattern[0][0] = 1;
            out_pattern[1][0] = 1;
            for(uint32_t loop = 1; loop < input2; loop++)
            {
                memcpy(&delay_array[loop], repeat_delay_array, SINGLE_SHOT_TRANSITIONS);
                memcpy(&(out_pattern[0][loop]), &one_click_pattern[0][loop],SINGLE_SHOT_TRANSITIONS);
                memcpy(&(out_pattern[1][loop]), &one_click_pattern[1][loop],SINGLE_SHOT_TRANSITIONS);
                        
            }
            break;
        }
        case MODE_BULB :
        {
//            bulb(input1, input2)
            number_of_transition = SINGLE_SHOT_TRANSITIONS;
            uint32_t local_delay_array[SINGLE_SHOT_TRANSITIONS + 1] = {SINGLE_SHOT_DURATION, LFCLK_TICKS_977((input1+input2) - 4), LFCLK_TICKS_977(4), FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
            memcpy(delay_array, local_delay_array, sizeof(local_delay_array));
            bool local_out_pattern[NUM_PIN_OUT][SINGLE_SHOT_TRANSITIONS + 1] = {
                {1, 0, 0, 0, 1},
                {1, 1, 0, 1, 1}};
            memcpy(out_pattern, local_out_pattern, sizeof(local_out_pattern));
            break;
        }
        case MODE_VIDEO :
        {
//            video(input1, input2)
            number_of_transition = VIDEO_TRANSITION;
            uint32_t local_delay_array[VIDEO_TRANSITION + 1] = {SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, LFCLK_TICKS_977(input1*1000), FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION, FOCUS_TRIGGER_TIME_DIFF, VIDEO_STOP_PULSE, FOCUS_TRIGGER_TIME_DIFF, SINGLE_SHOT_DURATION};
            memcpy(delay_array, local_delay_array, ARRAY_SIZE(local_delay_array));
            bool local_out_pattern[NUM_PIN_OUT][VIDEO_TRANSITION + 1] = {{1, 0, 0, 0, 1, 0, 0, 0, 1},
                {1, 1, 0, 1, 1, 1, 0 ,1 ,1}};
            memcpy(out_pattern, local_out_pattern, ARRAY_SIZE(local_out_pattern));           
            break;
        }
        case MODE_FOCUS :
        {
//            focus();
            number_of_transition = 2;
            uint32_t local_delay_array[3] = {SINGLE_SHOT_DURATION, SINGLE_SHOT_DURATION, SINGLE_SHOT_DURATION};
            memcpy(delay_array, local_delay_array, ARRAY_SIZE(local_delay_array));
            bool local_out_pattern[NUM_PIN_OUT][FOCUS_TRANSITIONS + 1] = {
                {1, 0, 1},
                {1, 1, 1}
            };
            memcpy(out_pattern, local_out_pattern, ARRAY_SIZE(local_out_pattern));           
            break;
        }
  
    }

    out_gen_start(number_of_transition, delay_array, out_pattern);
        
        
}
