/* 
 * File:   sensebe_rx_detect.c
 * Copyright (c) 2018 Appiko
 * Created on 29 October, 2018, 12:22 PM
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

#include "hal_gpio.h"
#include "ms_timer.h"
#include "out_pattern_gen.h"
#include "sensebe_rx_rev1.h"
#include "sensebe_rx_detect.h"
#include "log.h"
#include "led_ui.h"
#include "led_seq.h"
#include "ir_detect.h"
#include "device_tick.h"

#define MS_TIMER_USED MS_TIMER2
#define SINGLE_SHOT_TRANSITIONS 2
#define SINGLE_SHOT_DURATION MS_TIMER_TICKS_MS(250)
#define NULL_STATE 0
#define DETECT_FEEDBACK_TIMEOUT_TICKS MS_TIMER_TICKS_MS(600000)
#define INTER_TRIG_TIME MS_TIMER_TICKS_MS(750)

static bool detect_feedback_flag = true;
static uint32_t detect_time_pass = 0;
static uint32_t total_operation_time = INTER_TRIG_TIME + SINGLE_SHOT_DURATION + 10;
void out_gen_done_handler(uint32_t state)
{
    log_printf("%s\n", __func__);
    log_printf("State : %d\n", state);
}

static out_gen_config_t single_shot_config ={
    .num_transitions = SINGLE_SHOT_TRANSITIONS,
    .done_handler = out_gen_done_handler,
    .out_gen_state = NULL_STATE,
    .transitions_durations = { SINGLE_SHOT_DURATION, INTER_TRIG_TIME},
    .next_out ={
       {0, 1, 1},
       {0, 1, 1}
    },
};

void system_dowm (void)
{   
    log_printf("%s\n",__func__);
    ir_detect_stop ();
    NRF_POWER->SYSTEMOFF = (POWER_SYSTEMOFF_SYSTEMOFF_Enter 
        << POWER_SYSTEMOFF_SYSTEMOFF_Pos) & POWER_SYSTEMOFF_SYSTEMOFF_Msk;
}

void timer_1s (void);
void timer_200ms (void);

void timer_200ms (void)
{
    ir_detect_stop ();
    ms_timer_start (MS_TIMER_USED, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(1000),
                    timer_1s);
}

void timer_1s (void)
{
    ir_detect_pulse_detect ();
    ms_timer_start (MS_TIMER_USED, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(200),
                    timer_200ms);
}

void cam_trigger ()
{
//    log_printf("%s\n", __func__);
    if(out_gen_is_on () == false)
    {
        if(detect_feedback_flag == true)
        {
            led_ui_single_start (LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
        }
        else 
        {
            static uint32_t trig_count = 0, current_tick = 0, 
                previous_tick = 0;
            current_tick = ms_timer_get_current_count ();
            if(current_tick - previous_tick <= total_operation_time)
            {
                trig_count++;
            }
            else
            {
                trig_count = 0;
            }
            if(trig_count >= 30)
            {
                trig_count = 0;
                ir_detect_stop ();
                ms_timer_start (MS_TIMER_USED, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(1000), timer_1s);
            }
            previous_tick = current_tick;
        }
        out_gen_start (&single_shot_config);
    }
}

void sync_start ()
{
    log_printf ("%s\n", __func__);
    ms_timer_stop (MS_TIMER_USED);
    ir_detect_start ();
}

void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config)
{
    log_printf("%s\n", __func__);
    out_gen_init (sensebe_rx_detect_config->out_gen_no_of_pins,
                  sensebe_rx_detect_config->out_gen_pin_array,
                  sensebe_rx_detect_config->out_gen_init_val);

    ir_detect_config_t ir_detect_config = 
    {
        .detect_logic_level = false,
        .ir_missed_handler = cam_trigger,
        .ir_detect_handler = sync_start,
        .rx_en_pin = sensebe_rx_detect_config->rx_en_pin,
        .rx_in_pin = sensebe_rx_detect_config->rx_out_pin,
        .window_duration = sensebe_rx_detect_config->time_window_ms,
    };
    ir_detect_init (&ir_detect_config);
}

void sensebe_rx_detect_start (void)
{
    log_printf("%s\n", __func__);
    detect_time_pass = 0;
    detect_feedback_flag = true;
    ir_detect_start ();
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    ir_detect_stop ();
}

void sensebe_rx_detect_add_ticks (uint32_t interval)
{
    if(detect_feedback_flag == true)
    {
        detect_time_pass += interval;
        if(detect_time_pass >= DETECT_FEEDBACK_TIMEOUT_TICKS)
        {
            detect_feedback_flag = false;
            detect_time_pass = 0;
        }
    }
    
}
