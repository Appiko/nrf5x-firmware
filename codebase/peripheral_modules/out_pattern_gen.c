/*
 *  out_pattern_gen.c
 *
 *  Created on: 21-May-2018
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


#include "nrf.h"
#include "out_pattern_gen.h"
#include "nrf_assert.h"
#include "ms_timer.h"
#include "hal_gpio.h"
#include "string.h"
#include "log.h"

/** Specify the MS_TIMER used for the output pattern generator module */
#define OUT_GEN_MS_TIMER_USED           MS_TIMER1

static struct
{
    uint32_t num_out;
    bool is_on;
    uint8_t out_pins[OUT_GEN_MAX_NUM_OUT];
    bool next_out[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS];
    uint32_t num_transitions;
    uint32_t current_transition;
    uint32_t transitions_durations[OUT_GEN_MAX_TRANSITIONS];
    out_gen_state_t current_state;
}context;

void (*out_gen_done_handler)(out_gen_state_t out_gen_state);

static uint32_t timer_start_ticks_value;

static void timer_handler(void)
{
    context.current_transition++;


    for(uint32_t i = 0; i< context.num_out; i++)
    {
        hal_gpio_pin_write(context.out_pins[i], context.next_out[i][context.current_transition]);
    }

    if(context.current_transition == context.num_transitions)
    {
        context.is_on = false;
        out_gen_done_handler(context.current_state);
    }
    else
    {
        ms_timer_start(OUT_GEN_MS_TIMER_USED, MS_SINGLE_CALL,
                context.transitions_durations[context.current_transition],timer_handler);
    }
}

void out_gen_init(uint32_t num_out, uint32_t * out_pins)
{
    log_printf("OUT_GEN_INIT\n");
    ASSERT((num_out <= OUT_GEN_MAX_NUM_OUT) && (num_out > 0));

    context.num_out = num_out;
    for(uint32_t i = 0; i < num_out; i++)
    {
        context.out_pins[i] = out_pins[i];
    }

    context.is_on = false;
}

void out_gen_start(out_gen_config_t * out_gen_config)
{
    memset(context.next_out, 0, sizeof(context.next_out));
    memset(context.transitions_durations, 0 ,
            sizeof(context.transitions_durations));
    ASSERT((out_gen_config->num_transitions < OUT_GEN_MAX_TRANSITIONS) 
            && (out_gen_config->num_transitions > 0));

    context.num_transitions = out_gen_config->num_transitions;
    memcpy(context.transitions_durations, out_gen_config->transitions_durations,
            out_gen_config->num_transitions*sizeof(uint32_t) );
    for(uint32_t i = 0; i < context.num_out; i++)
    {
        memcpy( (*(context.next_out + i)), (*(out_gen_config->next_out+i)),
                (1+out_gen_config->num_transitions)*sizeof(bool));
        hal_gpio_pin_write(context.out_pins[i],
                out_gen_config->next_out[i][context.current_transition]);
    }
    context.is_on = true;
    context.current_transition = 0;
    context.current_state = out_gen_config->out_gen_state;
    out_gen_done_handler = out_gen_config->out_gen_done_handler;

    ms_timer_start(OUT_GEN_MS_TIMER_USED, MS_SINGLE_CALL,
            out_gen_config->transitions_durations[context.current_transition],timer_handler);
    timer_start_ticks_value = ms_timer_get_current_count();
}

void out_gen_stop(bool * out_vals)
{
    context.is_on = false;
    ms_timer_stop(OUT_GEN_MS_TIMER_USED);
    for(uint32_t i = 0; i< context.num_out; i++)
    {
        hal_gpio_pin_write(context.out_pins[i], out_vals[i]);
    }
}

bool out_gen_is_on(void)
{
    return context.is_on;
}

inline uint32_t out_gen_get_ticks(void)
{
    return ((ms_timer_get_current_count() + (1<<24) - timer_start_ticks_value)
            && 0x00FFFFFF);
}
