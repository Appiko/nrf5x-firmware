/*
 *  radio_trigger.c : Module to send trigger wireless-ly using radio peripheral.
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

#include "radio_trigger.h"
#include "hal_radio.h"
#include "nrf52810.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#include "nrf52810_bitfields.h"
#include "log.h"
#endif

/** @anchor Timer defines
 * @name Defines for the specific Timer peripheral used for radio trigger module
 * @{*/

#define TIMER_USED TIMER_USED_RADIO_TRIGGER
#define TIMER_ID CONCAT_2(NRF_TIMER,TIMER_USED)

#define TIMER_IRQN             TIMER_IRQN_a(TIMER_USED)
#define TIMER_IRQ_Handler      TIMER_IRQ_Handler_a(TIMER_USED)

#define TIMER_IRQN_a(n)        TIMER_IRQN_b(n)
#define TIMER_IRQN_b(n)        TIMER##n##_IRQn

#define TIMER_IRQ_Handler_a(n) TIMER_IRQ_Handler_b(n)
#define TIMER_IRQ_Handler_b(n) TIMER##n##_IRQHandler
/** @} */

#define TIMER_1MHz_PRESCALAR 4

#define TIMER_INTEN_OFFSET 16

#define TIMER_CHANNEL_COMMON_STARTUP TIMER_CHANNEL_USED_RADIO_TRIGGER_0

#define TIMER_CHANNEL_RX_ON TIMER_CHANNEL_USED_RADIO_TRIGGER_1

#define TIMER_CHANNEL_TX_ON TIMER_CHANNEL_USED_RADIO_TRIGGER_1
#define TIMER_CHANNEL_TX_FREQ TIMER_CHANNEL_USED_RADIO_TRIGGER_2

#define XTAL_STARTUP_TIME 450

#define MS_TO_US_CONV(n) (n * 1000)

//static uint32_t radio_rx_on_ticks = 0;

//static uint32_t radio_tx_on_ticks = 0;

static uint32_t radio_tx_freq_ticks = 0;

static radio_trigger_dir_t radio_dir = RADIO_TRIGGER_Tx;

volatile bool is_radio_free = true;

void (* p_radio_rx_handler) (void * p_data, uint32_t len);
void (* p_radio_tx_handler) (void * p_data, uint32_t len);
 
hal_radio_config_t radio_config; 

app_irq_priority_t radio_trig_irq_priority;

void radio_trigger_init (radio_trigger_init_t* radio_trig_init)
{
    p_radio_rx_handler = radio_trig_init->radio_trigger_rx_callback;
    p_radio_tx_handler = radio_trig_init->radio_trigger_tx_callback;
    radio_trig_irq_priority = radio_trig_init->irq_priority;
    radio_dir = radio_trig_init->comm_direction;
    {
        radio_config.freq = radio_trig_init->comm_freq;
        radio_config.irq_priority = radio_trig_init->irq_priority;
        radio_config.rx_done_handler = p_radio_rx_handler;
    }
    
    TIMER_ID->MODE = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos) & TIMER_MODE_MODE_Msk;
    
    TIMER_ID->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    
    TIMER_ID->PRESCALER = TIMER_1MHz_PRESCALAR;
    TIMER_ID->CC[TIMER_CHANNEL_COMMON_STARTUP] = XTAL_STARTUP_TIME;
    if(radio_trig_init->comm_direction == RADIO_TRIGGER_Tx)
    {
        TIMER_ID->INTENSET = 
            (1 << (TIMER_CHANNEL_COMMON_STARTUP + TIMER_INTEN_OFFSET)) |
            (1 << (TIMER_CHANNEL_TX_ON + TIMER_INTEN_OFFSET)) | 
            (1 << (TIMER_CHANNEL_TX_FREQ + TIMER_INTEN_OFFSET)) ;
        TIMER_ID->CC[TIMER_CHANNEL_TX_ON] = 
            (MS_TO_US_CONV(radio_trig_init->tx_on_time_ms) + XTAL_STARTUP_TIME);
        TIMER_ID->CC[TIMER_CHANNEL_TX_FREQ] = radio_trig_init->tx_on_freq_us+ XTAL_STARTUP_TIME;
        radio_tx_freq_ticks = radio_trig_init->tx_on_freq_us;
    }
    else
    {
        TIMER_ID->CC[TIMER_CHANNEL_RX_ON] = MS_TO_US_CONV(radio_trig_init->rx_on_time_ms) + XTAL_STARTUP_TIME;
        TIMER_ID->INTENSET = 
            (1 << (TIMER_CHANNEL_COMMON_STARTUP + TIMER_INTEN_OFFSET)) |
            (1 << (TIMER_CHANNEL_RX_ON + TIMER_INTEN_OFFSET));
    }
    
    
    NVIC_SetPriority (TIMER_IRQN, radio_trig_init->irq_priority);
    NVIC_EnableIRQ (TIMER_IRQN);
    
}

void radio_trigger_yell ()
{
    is_radio_free = false;
    TIMER_ID->CC[TIMER_CHANNEL_TX_FREQ] = radio_tx_freq_ticks + XTAL_STARTUP_TIME;

    TIMER_ID->TASKS_START = 1;
    hal_radio_init (&radio_config);
    NVIC_SetPriority (TIMER_IRQN, radio_trig_irq_priority);
    NVIC_EnableIRQ (TIMER_IRQN);
}

void radio_trigger_listen ()
{
    is_radio_free = false;
    TIMER_ID->TASKS_START = 1;
    hal_radio_init (&radio_config);
    NVIC_SetPriority (TIMER_IRQN, radio_trig_irq_priority);
    NVIC_EnableIRQ (TIMER_IRQN);
}

void radio_trigger_shut ()
{
    TIMER_ID->TASKS_STOP = 1;
    TIMER_ID->TASKS_SHUTDOWN = 1;
    NVIC_DisableIRQ (TIMER_IRQN);
    hal_radio_deinit ();
    is_radio_free = true;
}

#if ISR_MANAGER == true
void radio_trigger_timer_Handler ()
#else
void TIMER_IRQ_Handler ()
#endif
{
    if(radio_dir == RADIO_TRIGGER_Tx)
    {
        if(TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_COMMON_STARTUP] == true)
        {
#if ISR_MANAGER == false
            TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_COMMON_STARTUP] = false;
#endif
            log_printf("%s\n", __func__);
            hal_radio_start_tx ();
        }
        
        if(TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_TX_FREQ] == true)
        {
#if ISR_MANAGER == false
            TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_TX_FREQ] = false;
#endif
            hal_radio_start_tx ();
            TIMER_ID->CC[TIMER_CHANNEL_TX_FREQ] += radio_tx_freq_ticks;
            log_printf("CC[%d] : %d\n", TIMER_CHANNEL_TX_FREQ, TIMER_ID->CC[TIMER_CHANNEL_TX_FREQ]);
        }
        
        if(TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_TX_ON])
        {
#if ISR_MANAGER == false
            TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_TX_ON] = false;
#endif
            TIMER_ID->TASKS_CLEAR = 1;
            TIMER_ID->TASKS_STOP = 1;
            TIMER_ID->TASKS_SHUTDOWN = 1;
            (void) TIMER_ID->TASKS_SHUTDOWN;
            hal_radio_deinit ();
            is_radio_free = true;
        }
    }
    else
    {
        if(TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_COMMON_STARTUP] == true)
        {
#if ISR_MANAGER == false
            TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_COMMON_STARTUP] = false;
#endif
            hal_radio_start_rx ();
        }
        
        if(TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_RX_ON])
        {
#if ISR_MANAGER == false
            TIMER_ID->EVENTS_COMPARE[TIMER_CHANNEL_RX_ON] = false;
#endif
            TIMER_ID->TASKS_CLEAR = 1;
            TIMER_ID->TASKS_STOP = 1;
            TIMER_ID->TASKS_SHUTDOWN = 1;
            (void) TIMER_ID->TASKS_SHUTDOWN;
            hal_radio_deinit ();
            is_radio_free = true;
        }
    }
}

void radio_trigger_memorize_data (void * data, uint32_t len)
{
    hal_radio_set_tx_payload_data (data, len);
}

bool is_radio_trigger_availabel ()
{
    return is_radio_free;
}
