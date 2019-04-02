/* 
 * File:   tssp_ir_tx.c
 * Copyright (c) 2018 Appiko
 * Created on 5 February, 2019, 4:11 PM
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

#include "simple_pwm.h"
#include "tssp_ir_tx.h"
#include "hal_gpio.h"
#include "common_util.h"
#include "sys_config.h"


#define TIMER_ID_1KHZ CONCAT_2(NRF_TIMER, TIMER_USED_TSSP_IR_TX_1)
#define TIMER_ID_56KHZ CONCAT_2(NRF_TIMER, TIMER_USED_TSSP_IR_TX_2)

#define MAX_COUNT_CHANNEL TIMER_CHANNEL_USED_TSSP_IR_TX_2_2


#define ON_TIMER_GPIOTE_CHANNEL GPIOTE_CH_USED_TSSP_IR_TX_1
#define TIMER_56KHZ_GPIOTE_CHANNEL GPIOTE_CH_USED_TSSP_IR_TX_2

#define TIMERS_CHANNEL_USED TIMER_CHANNEL_USED_TSSP_IR_TX_1_1

#define PPI_56KHz_1 PPI_CH_USED_TSSP_IR_TX_1
#define PPI_56KHz_2 PPI_CH_USED_TSSP_IR_TX_2
#define PPI_xxKHz_1 PPI_CH_USED_TSSP_IR_TX_3
#define PPI_xxKHz_2 PPI_CH_USED_TSSP_IR_TX_4

static uint32_t tx_en;

void tssp_ir_tx_init (uint32_t tssp_tx_en, uint32_t tssp_tx_in)
{
    tx_en = tssp_tx_en;
    hal_gpio_cfg_output (tx_en,0);
    hal_gpio_cfg_output (tssp_tx_in,0);
    TIMER_ID_56KHZ->TASKS_STOP = 1;
    TIMER_ID_56KHZ->TASKS_CLEAR = 1;

    //TODO Enable assertion
    TIMER_ID_56KHZ->PRESCALER = TSSP_IR_TX_TIMER_FREQ_16MHz;

    TIMER_ID_56KHZ->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    TIMER_ID_56KHZ->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;

    TIMER_ID_56KHZ->CC[MAX_COUNT_CHANNEL] = 291;//291 IS FOR 56kHz @ 16MHz

    TIMER_ID_56KHZ->CC[TIMERS_CHANNEL_USED] = 29;
    TIMER_ID_56KHZ->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Enabled << TIMER_SHORTS_COMPARE3_CLEAR_Pos;

    TIMER_ID_56KHZ->EVENTS_COMPARE[MAX_COUNT_CHANNEL] = 0;
    

    TIMER_ID_56KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;

    NRF_GPIOTE->CONFIG[TIMER_56KHZ_GPIOTE_CHANNEL] =
              (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
            | (tssp_tx_in << GPIOTE_CONFIG_PSEL_Pos)
            | (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos)
            | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    NRF_PPI->CH[PPI_56KHz_1].EEP = (uint32_t) &(TIMER_ID_56KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED]);
    NRF_PPI->CH[PPI_56KHz_1].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_CLR[TIMER_56KHZ_GPIOTE_CHANNEL]);

    NRF_PPI->CH[PPI_56KHz_2].EEP = (uint32_t) &(TIMER_ID_56KHZ->EVENTS_COMPARE[MAX_COUNT_CHANNEL]);
    NRF_PPI->CH[PPI_56KHz_2].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_SET[TIMER_56KHZ_GPIOTE_CHANNEL]);

    
    TIMER_ID_1KHZ->MODE = TIMER_MODE_MODE_Timer;
    TIMER_ID_1KHZ->PRESCALER = TSSP_IR_TX_TIMER_FREQ_16MHz;
    TIMER_ID_1KHZ->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    TIMER_ID_1KHZ->CC[TIMERS_CHANNEL_USED] = 16000 * TSSP_IR_TX_ON_TIME_MS;

    TIMER_ID_1KHZ->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Enabled
                            << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
    TIMER_ID_1KHZ->SHORTS |= TIMER_SHORTS_COMPARE0_STOP_Enabled
                            << TIMER_SHORTS_COMPARE0_STOP_Pos;
  
    
    NRF_GPIOTE->CONFIG[ON_TIMER_GPIOTE_CHANNEL] = 
                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
                | ((tssp_tx_en << GPIOTE_CONFIG_PSEL_Pos)& GPIOTE_CONFIG_PSEL_Msk)
                | (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    
    NRF_PPI->CH[PPI_xxKHz_1].EEP = (uint32_t) &TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED];
    NRF_PPI->CH[PPI_xxKHz_1].TEP = (uint32_t) &TIMER_ID_1KHZ->TASKS_SHUTDOWN;
    
    NRF_PPI->FORK[PPI_xxKHz_1].TEP =  (uint32_t) &TIMER_ID_56KHZ->TASKS_SHUTDOWN;

    NRF_PPI->CH[PPI_xxKHz_2].EEP = (uint32_t) &TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED];
    NRF_PPI->CH[PPI_xxKHz_2].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_CLR[ON_TIMER_GPIOTE_CHANNEL]);

    
}

void tssp_ir_tx_start (void)
{
    TIMER_ID_56KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
    TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
    NRF_GPIOTE->TASKS_SET[ON_TIMER_GPIOTE_CHANNEL] = 1;
    TIMER_ID_1KHZ->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    TIMER_ID_1KHZ->TASKS_CLEAR = 1;
    
    NRF_PPI->CHENSET |= 1 << PPI_xxKHz_1;
    NRF_PPI->CHENSET |= 1 << PPI_xxKHz_2;
    NRF_PPI->CHENSET |= 1 << PPI_56KHz_1;
    NRF_PPI->CHENSET |= 1 << PPI_56KHz_2;

    TIMER_ID_1KHZ->TASKS_START = 1;

    TIMER_ID_56KHZ->TASKS_START = 1;
}

void tssp_ir_tx_stop (void)
{
    TIMER_ID_1KHZ->TASKS_SHUTDOWN = 1;
    hal_gpio_pin_clear (tx_en);
    
    NRF_PPI->CHENCLR |= 1 << PPI_xxKHz_1;
    NRF_PPI->CHENCLR |= 1 << PPI_xxKHz_2;
    NRF_PPI->CHENCLR |= 1 << PPI_56KHz_1;
    NRF_PPI->CHENCLR |= 1 << PPI_56KHz_2;
    TIMER_ID_56KHZ->TASKS_STOP = 1;

    TIMER_ID_56KHZ->TASKS_SHUTDOWN = 1;
}
