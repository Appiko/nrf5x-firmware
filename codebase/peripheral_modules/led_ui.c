/*
 *  led_ui.c
 *
 *  Created on: 26-Jun-2018
 *
 *  Copyright (c) 2018, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * A higher priority seq of any type takes precedence over a lower priority one
 * At the same priority a single seq takes precedence over a loop sequence
 * When a loop seq takes over a single seq, the single seq is_on is made false
 * When a single seq takes over a loop seq, the loop seq start again after the single seq is done
 * When a single seq is stopped, if there is a loop seq set then that takes over
 *
 */


#include "led_ui.h"
#include "hal_pwm.h"
#include "nrf_util.h"
#include "log.h"
#include "boards.h"
#include "stddef.h"

#define MAX_COUNT_PWM           1000
#define MAX_SEQ_LEN_SEC         20
#define PWM_UPDATE_PERIOD_MS    32
#define PWM_BUFFER_SIZE         ((uint32_t)((MAX_SEQ_LEN_SEC*1000)/PWM_UPDATE_PERIOD_MS))

static struct{
  led_sequences seq;
  bool is_on;
  uint16_t priority;
}led_ui_context[LED_UI_SEQ_T_SIZE];

static struct
{
  uint16_t color[LED_COLOR_MAX];
} seq_buffer[PWM_BUFFER_SIZE];

const uint16_t zero_val_arr[32] = { 0 };

void pwm_irq_handler(hal_pwm_irq_mask_t irq_source)
{
    if(HAL_PWM_IRQ_STOPPED_MASK == irq_source)
    {
        led_ui_context[LED_UI_SINGLE_SEQ].is_on = false;
        if(led_ui_context[LED_UI_LOOP_SEQ].is_on == true)
        {
            led_ui_loop_start(led_ui_context[LED_UI_LOOP_SEQ].seq,
                    led_ui_context[LED_UI_LOOP_SEQ].priority);
        }
    }
}

static bool check_single_seq(led_sequences seq, led_ui_priority_t priority, bool reset)
{
    //If the same single sequence is running and reset is false
    if((led_ui_context[LED_UI_SINGLE_SEQ].seq == seq) &&
     (led_ui_context[LED_UI_SINGLE_SEQ].is_on == true) && (reset == false))
    {
        return true;
    }

     //If a higher priority single sequence is running
     if((led_ui_context[LED_UI_SINGLE_SEQ].priority > priority)
                 && (led_ui_context[LED_UI_SINGLE_SEQ].is_on == true))
     {
         return true;
     }

     //If a higher priority loop sequence is running
     if((led_ui_context[LED_UI_LOOP_SEQ].priority > priority)
                      && (led_ui_context[LED_UI_LOOP_SEQ].is_on == true))
     {
         return true;
     }

     //Otherwise start the single sequence
     return false;
}

static bool check_loop_seq(led_sequences seq, led_ui_priority_t priority)
{
    //A higher priority single sequence is running
    if((led_ui_context[LED_UI_SINGLE_SEQ].is_on == true) &&
       (led_ui_context[LED_UI_SINGLE_SEQ].priority >= priority))
    {
        //Either loop sequence is not set or the set one is of lower priority
        if((led_ui_context[LED_UI_LOOP_SEQ].is_on == false) ||
           ((led_ui_context[LED_UI_LOOP_SEQ].priority < priority) &&
                   (led_ui_context[LED_UI_LOOP_SEQ].is_on == true)))
        {
            led_ui_context[LED_UI_LOOP_SEQ].is_on = true;
            led_ui_context[LED_UI_LOOP_SEQ].seq = seq;
            led_ui_context[LED_UI_LOOP_SEQ].priority = priority;
        }
        return true;
    }

    //A higher priority loop sequence is running
    if((led_ui_context[LED_UI_LOOP_SEQ].priority > priority)
                      && (led_ui_context[LED_UI_LOOP_SEQ].is_on == true))
    {
     return true;
    }

    //In case a lower priority single seq is usurped, take care of that
    if((led_ui_context[LED_UI_SINGLE_SEQ].is_on == true) &&
       (led_ui_context[LED_UI_SINGLE_SEQ].priority < priority))
    {
        led_ui_context[LED_UI_SINGLE_SEQ].is_on = false;
    }
    return false;
}

static void start_seq_pwm(led_sequences seq, led_ui_seq_t type, led_ui_priority_t priority)
{
    led_ui_context[type].is_on = true;
    led_ui_context[type].seq = seq;
    led_ui_context[type].priority = priority;

    uint16_t * dur_ptr = led_seq_get_seq_duration_ptr(seq);
    uint32_t seq_seg_num = led_seq_get_seg_len(seq);

    uint32_t led_num = led_seq_get_pin_num(seq);

    uint16_t * seq_ptr[LED_COLOR_MAX];
    uint32_t pin_arr[LED_COLOR_MAX];
    bool pin_idle[LED_COLOR_MAX];

    for(uint32_t k = 0; k < led_num; k++)
    {
        seq_ptr[k] = led_seq_get_seq_color_ptr(seq, k);
        pin_arr[k] = led_seq_get_pin_ptr()[k];
        pin_idle[k] = (!LEDS_ACTIVE_STATE);
    }
    for(uint32_t k = led_num; k < LED_COLOR_MAX; k++)
    {
        seq_ptr[k] = (uint16_t *)zero_val_arr;
        pin_arr[k] = led_seq_get_pin_ptr()[0];
        pin_idle[k] = (!LEDS_ACTIVE_STATE);
    }

    uint32_t overflow = 0; //To store the overflow from one segment to another
    uint32_t buff_cnt = 0; //To store the number of elements generated in buffer

    for(uint32_t i = 1; i<seq_seg_num; i++)
    {
        uint32_t curr_seg_dur = dur_ptr[i] - overflow;
        uint32_t seg_updates_num = 1 + curr_seg_dur/PWM_UPDATE_PERIOD_MS;

        for(uint32_t j = 0; j < seg_updates_num; j++)
        {
            uint32_t seg_count = overflow + j*PWM_UPDATE_PERIOD_MS;
            for(uint32_t k = 0; k < LED_COLOR_MAX; k++)
            {
                seq_buffer[buff_cnt].color[k] = ((int32_t)(seg_count *
                    ((int32_t)(seq_ptr[k][i] - seq_ptr[k][i-1]))))
                    /dur_ptr[i] + seq_ptr[k][i-1];

                seq_buffer[buff_cnt].color[k] = (LEDS_ACTIVE_STATE)?
                        (seq_buffer[buff_cnt].color[k] | (1<<15)):
                        (seq_buffer[buff_cnt].color[k]);
            }
            buff_cnt++;
        }

        uint32_t curr_dur_mod = curr_seg_dur - (seg_updates_num-1)*PWM_UPDATE_PERIOD_MS;
        overflow = (curr_dur_mod)?(PWM_UPDATE_PERIOD_MS - curr_dur_mod):0;
    }
    buff_cnt--;

    hal_pwm_init_t init_config =
    {
        .pins = pin_arr,
        .pin_idle_state = pin_idle,
        .pin_num = LED_COLOR_MAX,
        .oper_freq = HAL_PWM_FREQ_1MHz,
        .oper_mode = HAL_PWM_MODE_UP,
        .irq_priority = APP_IRQ_PRIORITY_MID
    };
    hal_pwm_init(&init_config);

    hal_pwm_start_t start_config =
    {
        .countertop = MAX_COUNT_PWM,
        .decoder_load = HAL_PWM_LOAD_GROUPED,
        .decoder_trigger = HAL_PWM_STEP_INTERNAL,
        .seq_config =
        {
            {
                .seq_values = (uint16_t *) seq_buffer,
                .len = (LED_COLOR_MAX * buff_cnt),
                .repeats = (PWM_UPDATE_PERIOD_MS - 1),
                .end_delay = 0
            },
            {
                .seq_values = (uint16_t *) seq_buffer,
                .len = (LED_COLOR_MAX * buff_cnt),
                .repeats = (PWM_UPDATE_PERIOD_MS - 1),
                .end_delay = 0
            }
        },
    };

    if(type == LED_UI_SINGLE_SEQ)
    {
        start_config.loop = 0;
        start_config.shorts_mask = HAL_PWM_SHORT_SEQEND0_STOP_MASK;
        start_config.interrupt_masks = HAL_PWM_IRQ_STOPPED_MASK;
        start_config.irq_handler = pwm_irq_handler;
    }
    else // LOOP_SEQ
    {
        start_config.loop = 1;
        start_config.shorts_mask = HAL_PWM_SHORT_LOOPSDONE_SEQSTART0_MASK;
        start_config.interrupt_masks = 0;
        start_config.irq_handler = NULL;
    }

    hal_pwm_start(&start_config);
}

void led_ui_single_start(led_sequences seq, led_ui_priority_t priority, bool reset)
{

    if(check_single_seq(seq, priority, reset))
    {
        return;
    }

    start_seq_pwm(seq, LED_UI_SINGLE_SEQ, priority);
}

void led_ui_loop_start(led_sequences seq, led_ui_priority_t priority)
{
    if(check_loop_seq(seq, priority))
    {
        return;
    }

    start_seq_pwm(seq, LED_UI_LOOP_SEQ, priority);
}

static void stop_seq_type(led_ui_seq_t type)
{
    //In case where single is running and loop needs to stop, don't call pwm_stop
    if((LED_UI_SINGLE_SEQ == type) || ((LED_UI_LOOP_SEQ == type)
            && (led_ui_context[LED_UI_SINGLE_SEQ].is_on == false)))
    {
        hal_pwm_stop();
    }
    led_ui_context[type].is_on = false;

    //This condition will only be true for type == LED_UI_SINGLE_SEQ
    if(led_ui_context[LED_UI_LOOP_SEQ].is_on == true)
    {
        led_ui_loop_start(led_ui_context[LED_UI_LOOP_SEQ].seq,
                led_ui_context[LED_UI_LOOP_SEQ].priority);
    }
}

void led_ui_type_stop_all(led_ui_seq_t type)
{
    if(led_ui_context[type].is_on == true)
    {
        stop_seq_type(type);
    }
}

void led_ui_stop_seq(led_ui_seq_t type, led_sequences seq)
{
    if((led_ui_context[type].is_on == true) &&
            (led_ui_context[type].seq == seq))
    {
        stop_seq_type(type);
    }
}

void led_ui_stop_priority(led_ui_seq_t type, uint32_t priority)
{
    if((led_ui_context[type].is_on == true) &&
            (led_ui_context[type].priority == priority))
    {
        stop_seq_type(type);
    }
}

void led_ui_stop_everything(void)
{
    hal_pwm_stop();
    led_ui_context[LED_UI_SINGLE_SEQ].is_on = false;
    led_ui_context[LED_UI_LOOP_SEQ].is_on = false;
}

led_sequences led_ui_get_current_seq(led_ui_seq_t type)
{
    return led_ui_context[type].seq;
}

