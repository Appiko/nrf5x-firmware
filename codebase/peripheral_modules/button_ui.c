/*
 *  button_ui.c
 *
 *  Created on: 25-Apr-2018
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

#include "button_ui.h"
#include "hal_gpio.h"

#ifndef BUTTON_ACTIVE_STATE
#error "The board definition file must specify the GPIO state on button press"
#endif

#if BUTTON_ACTIVE_STATE == 1
#define GPIO_PULL_RESISTOR  HAL_GPIO_PULL_DOWN
#define GPIO_PIN_SENSE      GPIO_PIN_CNF_SENSE_High
#define BUTTON_PRESSED      1
#define BUTTON_RELEASED     0
#else
#define GPIO_PULL_RESISTOR  HAL_GPIO_PULL_UP
#define GPIO_PIN_SENSE      GPIO_PIN_CNF_SENSE_Low
#define BUTTON_PRESSED      0
#define BUTTON_RELEASED     1
#endif

uint32_t btn_pin;
void (*handler)(button_ui_steps step, button_ui_action act);

void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    (void) NRF_GPIOTE->EVENTS_PORT;

    handler(BUTTON_UI_WAKE);
}

void button_ui_init(uint32_t button_pin, uint32_t irq_priority,
        void (*button_ui_handler)(button_ui_steps step))
{
    hal_gpio_cfg(button_pin, GPIO_PIN_CNF_DIR_Input,
        GPIO_PIN_CNF_INPUT_Connect, GPIO_PULL_RESISTOR,
        GPIO_PIN_CNF_DRIVE_S0S1, GPIO_PIN_SENSE);

    btn_pin = button_pin;
    handler = button_ui_handler;

    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_PORT_Enabled
            << GPIOTE_INTENSET_PORT_Pos);

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, irq_priority);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

bool button_ui_add_tick(uint32_t ui_ticks)
{
    static uint32_t ticks = 0, step = 0;

    volatile bool button_state = button_check();

    if((hal_gpio_pin_read(btn_pin) == BUTTON_RELEASED)
            && (0 == step))
    {
        ticks = 0;
        return false;
    }

    if((hal_gpio_pin_read(btn_pin) == BUTTON_PRESSED)){
        ticks += ui_ticks;
    }

    if(ticks > press_duration[step])
    {
        rgb_sequence_loop_start(led_seq_to_set[step], RGB_SEQ_HIGHEST_PRIORITY);
        step++;
    }

    if(button_all_released())
    {
        uint32_t temp_step = step;
        step = ticks = 0;
        rgb_sequence_stop_everything();
        handler(temp_step);
    }

    return true;
}

void button_ui_config_wake(bool set_wake_on)
{
    if (set_wake_on)
    {
        NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_PORT_Enabled
                << GPIOTE_INTENSET_PORT_Pos);
    }
    else
    {
        NRF_GPIOTE->INTENCLR = (GPIOTE_INTENCLR_PORT_Enabled
                << GPIOTE_INTENCLR_PORT_Pos);
    }
}

