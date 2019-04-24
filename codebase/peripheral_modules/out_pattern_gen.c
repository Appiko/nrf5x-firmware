/**
 *  out_pattern_gen.c : Output Pattern Generator
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


#include "nrf.h"
#include "out_pattern_gen.h"
#include "nrf_assert.h"
#include "ms_timer.h"
#include "hal_gpio.h"
#include "string.h"
#include "log.h"
#include "stddef.h"
#include "common_util.h"

/** Specify the MS_TIMER used for the output pattern generator module */
#define OUT_GEN_MS_TIMER_USED           CONCAT_2(MS_TIMER, MS_TIMER_USED_OUT_GEN) 

static struct
{
    uint32_t num_out;
    bool is_on;
    uint8_t out_pins[OUT_GEN_MAX_NUM_OUT];
    bool next_out[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS];
    uint32_t num_transitions;
    uint32_t current_transition;
    uint32_t transitions_durations[OUT_GEN_MAX_TRANSITIONS];
    uint32_t end_context;
}context;

void (*done_handler)(uint32_t out_gen_state);

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
        if(done_handler != NULL)
        {
            done_handler(context.end_context);
        }
    }
    else
    {
        ms_timer_start(OUT_GEN_MS_TIMER_USED, MS_SINGLE_CALL,
                context.transitions_durations[context.current_transition],timer_handler);
    }
}

void out_gen_init(uint32_t num_out, uint32_t * out_pins, bool * out_init_value)
{
    log_printf("OUT_GEN_INIT\n");
    ASSERT((num_out <= OUT_GEN_MAX_NUM_OUT) && (num_out > 0));

    context.num_out = num_out;
    for(uint32_t i = 0; i < num_out; i++)
    {
        context.out_pins[i] = out_pins[i];
        hal_gpio_cfg_output(out_pins[i], out_init_value[i]);
    }

    context.is_on = false;
}

void out_gen_start(out_gen_config_t * out_gen_config)
{
    ASSERT((out_gen_config->num_transitions < OUT_GEN_MAX_TRANSITIONS)
            && (out_gen_config->num_transitions > 0));
    memset(context.next_out, 0, sizeof(context.next_out));
    memset(context.transitions_durations, 0 ,
            sizeof(context.transitions_durations));

    context.current_transition = 0;
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
    context.end_context = out_gen_config->out_gen_state;
    done_handler = out_gen_config->done_handler;

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
            & 0x00FFFFFF);
}
