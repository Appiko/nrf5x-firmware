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
}context;

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
    }
    else
    {
        ms_timer_start(OUT_GEN_MS_TIMER_USED, MS_SINGLE_CALL,
                context.transitions_durations[context.current_transition],timer_handler);
    }
}

void out_gen_init(uint32_t num_out, uint32_t * out_pins)
{
    ASSERT((num_out <= OUT_GEN_MAX_NUM_OUT) && (num_out > 0));

    context.num_out = num_out;
    for(uint32_t i = 0; i < num_out; i++)
    {
        context.out_pins[i] = out_pins[i];
    }

    context.is_on = false;
}

void out_gen_start(uint32_t num_transitions, uint32_t * transitions_durations,
        bool next_out[][OUT_GEN_MAX_TRANSITIONS])
{
    ASSERT((num_transitions < OUT_GEN_MAX_TRANSITIONS) && (num_transitions > 0));

    context.num_transitions = num_transitions;
    memcpy(context.transitions_durations, transitions_durations,
            num_transitions*sizeof(uint32_t) );

    for(uint32_t i = 0; i < context.num_out; i++)
    {
        memcpy( (*(context.next_out + i)), (*(next_out+i)),
                (1+num_transitions)*sizeof(bool));
        hal_gpio_pin_write(context.out_pins[i],
                next_out[i][context.current_transition]);
    }

    context.is_on = true;
    context.current_transition = 0;

    ms_timer_start(OUT_GEN_MS_TIMER_USED, MS_SINGLE_CALL,
            transitions_durations[context.current_transition],timer_handler);
}

void out_gen_stop(bool * out_vals)
{
    ms_timer_stop(OUT_GEN_MS_TIMER_USED);
    for(uint32_t i = 0; i< context.num_out; i++)
    {
        hal_gpio_pin_write(context.out_pins[i], out_vals[i]);
    }
    context.is_on = false;
}

bool out_gen_is_on(void)
{
    return context.is_on;
}
