/**
 *  tssp_ir_tx.c : IR TX module compatible with TSSP sensors
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
#include "tssp_ir_tx.h"
#include "hal_gpio.h"
#include "common_util.h"
#include "sys_config.h"
#include "nrf_util.h"

#if ISR_MANAGER == 1
#include "template_isr_manage.h"
#endif

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

static uint32_t tx_in;

void tssp_ir_tx_init (uint32_t tssp_tx_en, uint32_t tssp_tx_in)
{
    tx_en = tssp_tx_en;
    tx_in = tssp_tx_in;
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

//    TIMER_ID_1KHZ->SHORTS |= TIMER_SHORTS_COMPARE0_CLEAR_Enabled
//                            << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
//    TIMER_ID_1KHZ->SHORTS |= TIMER_SHORTS_COMPARE0_STOP_Enabled
//                            << TIMER_SHORTS_COMPARE0_STOP_Pos;
//  
//    
//    NRF_GPIOTE->CONFIG[ON_TIMER_GPIOTE_CHANNEL] = 
//                (GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos)
//                | ((tssp_tx_en << GPIOTE_CONFIG_PSEL_Pos)& GPIOTE_CONFIG_PSEL_Msk)
//                | (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
//                | (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
//
//    
//    NRF_PPI->CH[PPI_xxKHz_1].EEP = (uint32_t) &TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED];
//    NRF_PPI->CH[PPI_xxKHz_1].TEP = (uint32_t) &TIMER_ID_1KHZ->TASKS_SHUTDOWN;
//    
//    NRF_PPI->FORK[PPI_xxKHz_1].TEP =  (uint32_t) &TIMER_ID_56KHZ->TASKS_SHUTDOWN;
//
//    NRF_PPI->CH[PPI_xxKHz_2].EEP = (uint32_t) &TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED];
//    NRF_PPI->CH[PPI_xxKHz_2].TEP = (uint32_t) &(NRF_GPIOTE->TASKS_CLR[ON_TIMER_GPIOTE_CHANNEL]);
    
    NVIC_SetPriority (TIMER2_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_EnableIRQ (TIMER2_IRQn);
}
#if ISR_MANAGER == 1
void tssp_ir_tx_timer1_Handler ()
#else
void TIMER2_IRQHandler ()
#endif
{
    hal_gpio_pin_clear (tx_en);
    hal_gpio_pin_clear (tx_in);
#if ISR_MANAGER == 0
    TIMER_ID_56KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
    TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
#endif
    TIMER_ID_1KHZ->TASKS_CLEAR = 1;
    TIMER_ID_1KHZ->TASKS_STOP = 1;
    TIMER_ID_1KHZ->TASKS_SHUTDOWN = 1;
    TIMER_ID_56KHZ->TASKS_SHUTDOWN = 1;
}

void tssp_ir_tx_timer2_Handler ()
{
    
}

void tssp_ir_tx_start (void)
{
    hal_gpio_pin_set (tx_en);
    TIMER_ID_56KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
    TIMER_ID_1KHZ->EVENTS_COMPARE[TIMERS_CHANNEL_USED] = 0;
    NRF_GPIOTE->TASKS_SET[ON_TIMER_GPIOTE_CHANNEL] = 1;
    TIMER_ID_1KHZ->INTENSET |= TIMER_INTENSET_COMPARE0_Msk;
    TIMER_ID_1KHZ->TASKS_CLEAR = 1;
    
//    NRF_PPI->CHENSET |= 1 << PPI_xxKHz_1;
//    NRF_PPI->CHENSET |= 1 << PPI_xxKHz_2;
    NRF_PPI->CHENSET |= 1 << PPI_56KHz_1;
    NRF_PPI->CHENSET |= 1 << PPI_56KHz_2;
    TIMER_ID_56KHZ->TASKS_START = 1;
    TIMER_ID_1KHZ->TASKS_START = 1;

}

void tssp_ir_tx_stop (void)
{
    TIMER_ID_1KHZ->TASKS_SHUTDOWN = 1;
    hal_gpio_pin_clear (tx_en);
    
//    NRF_PPI->CHENCLR |= 1 << PPI_xxKHz_1;
//    NRF_PPI->CHENCLR |= 1 << PPI_xxKHz_2;
    NRF_PPI->CHENCLR |= 1 << PPI_56KHz_1;
    NRF_PPI->CHENCLR |= 1 << PPI_56KHz_2;
    TIMER_ID_56KHZ->TASKS_STOP = 1;

    TIMER_ID_56KHZ->TASKS_SHUTDOWN = 1;
}
