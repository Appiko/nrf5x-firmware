/*
 *  simple_pwm.c
 *
 *  Created on: 20-Jun-2017
 *
 *  Copyright (c) 2017, Appiko
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

#include "simple_pwm.h"
#include "hal_gpio.h"
#include "nrf.h"
#include "common_util.h"
#include "log.h"
#include "nrf_util.h"

/** @anchor simple_pwm_defines
 * @name Defines for the specific RTC peripheral used for ms timer
 * @{*/
#define TIMER_ID              CONCAT_2(NRF_TIMER, SIMPLE_PWM_TIMER_USED)
#define TIMER_IRQN            CONCAT_3(TIMER, SIMPLE_PWM_TIMER_USED, _IRQn)
#define TIMER_IRQ_Handler     CONCAT_3(TIMER, SIMPLE_PWM_TIMER_USED, _IRQHandler)
/** @} */


uint32_t pwm_pins[3];


void simple_pwm_init(simple_pwm_config * config)
{
    TIMER_ID->TASKS_STOP = 1;
    TIMER_ID->TASKS_CLEAR = 1;

    //TODO Enable assertion
    //ASSERT((config->freq < 10) && (config->freq >= 0));
    TIMER_ID->PRESCALER = config->freq;

    //ASSERT((config->max_count > 2) && (config->max_count < 65536));
    TIMER_ID->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    TIMER_ID->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

    TIMER_ID->CC[SIMPLE_PWM_MAX_CHANNEL] = config->max_count;

    TIMER_ID->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Enabled << TIMER_SHORTS_COMPARE3_CLEAR_Pos;

    TIMER_ID->EVENTS_COMPARE[SIMPLE_PWM_MAX_CHANNEL] = 0;
    for(uint32_t i; i<SIMPLE_PWM_MAX_CHANNEL; i++)
    {
        TIMER_ID->EVENTS_COMPARE[i] = 0;
        hal_gpio_cfg_output(config->pin[i],0);
        pwm_pins[i] = config->pin[i];

        NRF_PPI->CH[2*i].EEP = (uint32_t) &(TIMER_ID->EVENTS_COMPARE[i]);
        NRF_PPI->CH[2*i].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_SET[i+SIMPLE_PWM_GPIOTE_START_CH]);

        NRF_PPI->CH[2*i+1].EEP = (uint32_t) &(TIMER_ID->EVENTS_COMPARE[SIMPLE_PWM_MAX_CHANNEL]);
        NRF_PPI->CH[2*i+1].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_CLR[i+SIMPLE_PWM_GPIOTE_START_CH]);
    }

    NRF_PPI->CHENSET = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5);

    TIMER_ID->TASKS_START = 1;
}

void simple_pwm_set_value(simple_pwm_channels channel, uint32_t value)
{
    if(value >= TIMER_ID->CC[SIMPLE_PWM_MAX_CHANNEL])
    {
        NRF_GPIOTE->CONFIG[channel+SIMPLE_PWM_GPIOTE_START_CH] =
                  (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos)
                | (pwm_pins[channel] << GPIOTE_CONFIG_PSEL_Pos)
                | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
        hal_gpio_pin_set(pwm_pins[channel]);
    }
    else if(value == 0)
    {
        NRF_GPIOTE->CONFIG[channel+SIMPLE_PWM_GPIOTE_START_CH] =
                  (GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos)
                | (pwm_pins[channel] << GPIOTE_CONFIG_PSEL_Pos)
                | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
        hal_gpio_pin_clear(pwm_pins[channel]);
    }
    else
    {
        NRF_GPIOTE->CONFIG[channel+SIMPLE_PWM_GPIOTE_START_CH] =
                  (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
                | (pwm_pins[channel] << GPIOTE_CONFIG_PSEL_Pos)
                | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
        TIMER_ID->CC[channel] = value;
    }
}
