/**
 *  simple_pwm.c : A simple PWM driver
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


void simple_pwm_init(simple_pwm_timer_freq_t freq,uint32_t max_count)
{
    TIMER_ID->TASKS_STOP = 1;
    TIMER_ID->TASKS_CLEAR = 1;

    //TODO Enable assertion
    //ASSERT((config->freq < 10) && (config->freq >= 0));
    TIMER_ID->PRESCALER = freq;

    //ASSERT((config->max_count > 2) && (config->max_count < 65536));
    TIMER_ID->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    TIMER_ID->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

    TIMER_ID->CC[SIMPLE_PWM_MAX_CHANNEL] = max_count;

    TIMER_ID->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Enabled << TIMER_SHORTS_COMPARE3_CLEAR_Pos;

    TIMER_ID->EVENTS_COMPARE[SIMPLE_PWM_MAX_CHANNEL] = 0;
}

void simple_pwm_channel_setup(simple_pwm_channel_t channel, uint32_t pwm_out_pin, 
                              uint32_t value)
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
        TIMER_ID->EVENTS_COMPARE[channel] = 0;
        hal_gpio_cfg_output(pwm_out_pin,0);
        pwm_pins[channel] = pwm_out_pin;

        NRF_PPI->CH[2*channel].EEP = (uint32_t) &(TIMER_ID->EVENTS_COMPARE[channel]);
        NRF_PPI->CH[2*channel].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_CLR[channel+SIMPLE_PWM_GPIOTE_START_CH]);

        NRF_PPI->CH[2*channel+1].EEP = (uint32_t) &(TIMER_ID->EVENTS_COMPARE[SIMPLE_PWM_MAX_CHANNEL]);
        NRF_PPI->CH[2*channel+1].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_SET[channel+SIMPLE_PWM_GPIOTE_START_CH]);
        
        NRF_GPIOTE->CONFIG[channel+SIMPLE_PWM_GPIOTE_START_CH] =
                  (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
                | (pwm_pins[channel] << GPIOTE_CONFIG_PSEL_Pos)
                | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
        TIMER_ID->CC[channel] = value;
    }
}

void simple_pwm_start ()
{
    NRF_PPI->CHENSET = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5);

    TIMER_ID->TASKS_START = 1;
}

void simple_pwm_stop ()
{
    NRF_PPI->CHENCLR = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5);
    
    TIMER_ID->TASKS_STOP = 1;

    TIMER_ID->TASKS_SHUTDOWN = 1;
}