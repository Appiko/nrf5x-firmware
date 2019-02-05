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

#define TIMER_ID CONCAT_2(NRF_TIMER, TSSP_TX_TIMER_USED)

#define TIMER_CHANNEL_USED 0


uint32_t tx_en;
void tssp_ir_tx_init (uint32_t tssp_tx_en, uint32_t tssp_tx_in)
{
    tx_en = tssp_tx_en;
    simple_pwm_init (SIMPLE_PWM_TIMER_FREQ_16MHz, 291);
    simple_pwm_channel_setup (SIMPLE_PWM_CHANNEL0, tssp_tx_in, 29);
    hal_gpio_cfg_output (tx_en,0);
    
    TIMER_ID->MODE = TIMER_MODE_MODE_Timer;
    TIMER_ID->PRESCALER = SIMPLE_PWM_TIMER_FREQ_16MHz;
    TIMER_ID->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    TIMER_ID->CC[TIMER_CHANNEL_USED] = 16000 * TSSP_IR_TX_ON_TIME_MS;
    TIMER_ID->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Enabled
                            << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
    TIMER_ID->SHORTS |= TIMER_SHORTS_COMPARE0_STOP_Enabled
                            << TIMER_SHORTS_COMPARE0_STOP_Pos;
    NRF_PPI->CH[TSSP_TX_PPI_CHANNEL_USED].EEP = (uint32_t) &TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_USED];
    NRF_PPI->CH[TSSP_TX_PPI_CHANNEL_USED].TEP = (uint32_t) &TIMER_ID->TASKS_SHUTDOWN;
    
    //TODO : Figure out way to avoid this
    NRF_PPI->FORK[TSSP_TX_PPI_CHANNEL_USED].TEP =  (uint32_t) &NRF_TIMER1->TASKS_SHUTDOWN;
    
    
}

void tssp_ir_tx_start (void)
{
    TIMER_ID->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    TIMER_ID->TASKS_CLEAR = 1;
    
    NRF_PPI->CHENSET |= 1 << TSSP_TX_PPI_CHANNEL_USED;

    TIMER_ID->TASKS_START = 1;
    simple_pwm_start ();
}

void tssp_ir_tx_stop (void)
{
    TIMER_ID->TASKS_SHUTDOWN;
    hal_gpio_pin_clear (tx_en);
    simple_pwm_stop ();
}
