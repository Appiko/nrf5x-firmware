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

#define MS_TIMER_USED MS_TIMER2
#define SINGLE_SHOT_TRANSITIONS 1
#define SINGLE_SHOT_DURATION MS_TIMER_TICKS_MS(250)
#define NULL_STATE 0

static uint32_t no_output_pins = 2;
static uint32_t cam_trigger_pin_array[] = {JACK_FOCUS_PIN, JACK_TRIGGER_PIN};
static bool cam_trigger_defaults[] = {1, 1};

void out_gen_done_handler(uint32_t state)
{
    log_printf("%s\n", __func__);
    log_printf("State : %d\n", state);
    return;
}

static out_gen_config_t single_shot_config ={
    .num_transitions = SINGLE_SHOT_TRANSITIONS,
    .done_handler = out_gen_done_handler,
    .out_gen_state = NULL_STATE,
    .transitions_durations = { SINGLE_SHOT_DURATION},
    .next_out ={
       {0, 1},
       {0, 1}
    },
};



void cam_trigger ()
{
    log_printf("%s\n", __func__);
    if(out_gen_is_on () == false)
    {
        led_ui_single_start (LED_SEQ_PIR_PULSE, LED_UI_HIGH_PRIORITY, true);
        out_gen_start (&single_shot_config);
    }
}

void timer_handler ()
{
//    log_printf("%s\n", __func__);
    static uint32_t count_ms = 0;
    count_ms++;
    if(count_ms == 100)
    {
        cam_trigger ();
        count_ms = 0;
        return;
    }
    if(hal_gpio_pin_read (TSSP_RX_OUT) == 0)
    {
        count_ms= 0;
    }
}

void sensebe_rx_detect_init (void)
{
    log_printf("%s\n", __func__);
    hal_gpio_cfg_input (TSSP_RX_OUT, HAL_GPIO_PULL_UP);
    hal_gpio_cfg_output (TSSP_RX_EN, 0);
    out_gen_init (no_output_pins, cam_trigger_pin_array, cam_trigger_defaults);
}

void sensebe_rx_detect_start (void)
{
    log_printf("%s\n", __func__);
    ms_timer_start (MS_TIMER_USED,MS_REPEATED_CALL, MS_TIMER_TICKS_MS(1),timer_handler);
    hal_gpio_pin_set (TSSP_RX_EN);
}

void sensebe_rx_detect_stop (void)
{
    log_printf("%s\n", __func__);
    ms_timer_stop (MS_TIMER_USED);
    hal_gpio_pin_clear (TSSP_RX_EN);
}