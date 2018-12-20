/* 
 * File:   ir_detect.c
 * Copyright (c) 2018 Appiko
 * Created on 18 December, 2018, 3:27 PM
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

#include "ir_detect.h"
#include "hal_gpio.h"
#include "common_util.h"
#include "hal_clocks.h"
#include "nrf_util.h"
#include "stddef.h"
#include "log.h"


#ifndef IR_DETECT_FREQ
#ifdef MS_TIMER_FREQ
#define IR_DETECT_FREQ MS_TIMER_FREQ
#else
#define IR_DETECT_FREQ 32768
#endif
#endif

/** Channel 2 of RTC0 is used here */
#define RTC_CHANNEL_USED 2
/** Channel 2 of GPIOTE is used here */
#define GPIOTE_CHANNEL_USED 2

/** Channel 1 of PPI is used here */
#define PPI_CHANNEL_USED 1

/** Pin number of Enable pin present on IR Rx module */
uint32_t ir_en_pin;

/** Function pointer which is to be called if it doesn't detect any pulse in\
 *  given window of time */
void (*missed_handler)(void);

void ir_detect_init (ir_detect_config_t * ir_detect_config)
{
    ir_en_pin = ir_detect_config->rx_en_pin;
    hal_gpio_cfg_input (ir_detect_config->rx_in_pin,
                        HAL_GPIO_PULL_UP);

    if(ir_detect_config->ir_missed_handler != NULL)
    {
        missed_handler = ir_detect_config->ir_missed_handler;
        IR_DETECT_RTC_USED->PRESCALER = ROUNDED_DIV(LFCLK_FREQ, IR_DETECT_FREQ) - 1;
        IR_DETECT_RTC_USED->CC[RTC_CHANNEL_USED] = ir_detect_config->window_duration;
        
        
        IR_DETECT_RTC_USED->INTENSET |= 1 << (RTC_CHANNEL_USED+16);
        
        NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
        NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] =
            GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos|
            GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos| 
            ((ir_detect_config->rx_in_pin << GPIOTE_CONFIG_PSEL_Pos) 
             & GPIOTE_CONFIG_PSEL_Msk);
        
        
        
        NVIC_SetPriority (RTC0_IRQn, 1);
        NVIC_EnableIRQ (RTC0_IRQn);
        NVIC_ClearPendingIRQ(RTC0_IRQn);
        NRF_PPI->CH[PPI_CHANNEL_USED].EEP = (uint32_t) &NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED];
        NRF_PPI->CH[PPI_CHANNEL_USED].TEP = (uint32_t) &IR_DETECT_RTC_USED->TASKS_CLEAR;
    }
    
}

void ir_detect_start ()
{
    hal_gpio_cfg_output (ir_en_pin, 1);
    NRF_PPI->CHENSET |= 1 << PPI_CHANNEL_USED;
    IR_DETECT_RTC_USED->TASKS_CLEAR = 1;
    (void) IR_DETECT_RTC_USED->TASKS_CLEAR;
    IR_DETECT_RTC_USED->TASKS_START = 1;
}

void ir_detect_stop ()
{
    hal_gpio_cfg_output (ir_en_pin, 0);
    NRF_PPI->CHENCLR |= 1 << PPI_CHANNEL_USED;
    IR_DETECT_RTC_USED->TASKS_CLEAR = 1;
    (void) IR_DETECT_RTC_USED->TASKS_CLEAR;
    IR_DETECT_RTC_USED->TASKS_STOP = 1;
}


void RTC0_IRQHandler (void)
{
    
    IR_DETECT_RTC_USED->EVENTS_COMPARE[RTC_CHANNEL_USED] = 0;
    (void) IR_DETECT_RTC_USED->EVENTS_COMPARE[RTC_CHANNEL_USED];
    missed_handler ();
    IR_DETECT_RTC_USED->TASKS_CLEAR = 1;
    (void) IR_DETECT_RTC_USED->TASKS_CLEAR;
    IR_DETECT_RTC_USED->TASKS_START = 1;

}
