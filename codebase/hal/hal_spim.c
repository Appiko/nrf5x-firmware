/* 
 * File:   hal_spim.c
 * Copyright (c) 2018 Appiko
 * Created on 14 March, 2019, 3:24 PM
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

#include <stddef.h>

#include "hal_spim.h"
#include "common_util.h"
#include "hal_gpio.h"
#include "stddef.h"
#include "nrf_assert.h"
#include "nrf_util.h"
#include "log.h"

/** @anchor twim_defines
 * @name Defines for the specific SPI peripheral used 
 * @{*/
#define SPIM_ID CONCAT_2(NRF_SPIM,SPIM_USED)

#define SPIM_IRQN SPIM_IRQN_a(SPIM_USED)
#define SPIM_IRQ_Handler SPIM_IRQ_Handler_a(SPIM_USED)

#define SPIM_IRQ_Handler_a(n) SPIM_IRQ_Handler_b(n)
#define SPIM_IRQ_Handler_b(n) SPIM##n##_SPIS##n##_TWIM##n##_TWIS##n##_SPI##n##_TWI##n##_IRQHandler

#define SPIM_IRQN_a(n)  SPIM_IRQN_b(n)
#define SPIM_IRQN_b(n)  SPIM##n##_SPIS##n##_TWIM##n##_TWIS##n##_SPI##n##_TWI##n##_IRQn
/** @} */

/** variable to store CS Bar pin number */
static uint32_t csBar = 0;

/** Variable to store interrupts which are to be enabled */
static uint32_t intr_enabled = 0;

/** Status flag */
static bool mod_is_available = true;

/** Function pointer buffers */
void (*rx_done) (uint32_t last_byte_no);
void (*tx_done) (uint32_t last_byte_no);

void hal_spim_init (hal_spim_init_t * spim_init)
{
    hal_gpio_cfg_output (spim_init->csBar_pin, 1);
    SPIM_ID->TASKS_SUSPEND = 1;
    SPIM_ID->TASKS_STOP = 1;
    SPIM_ID->CONFIG = spim_init->logic_clk_order_cnf;
    SPIM_ID->FREQUENCY = spim_init->freq;
    SPIM_ID->PSEL.MISO = spim_init->miso_pin;
    SPIM_ID->PSEL.MOSI = spim_init->mosi_pin;
    SPIM_ID->PSEL.SCK = spim_init->sck_pin;
    csBar = spim_init->csBar_pin;
    intr_enabled = spim_init->en_intr;
    SPIM_ID->INTENSET = spim_init->en_intr | SPIM_INTENSET_END_Msk;
    NVIC_SetPriority (SPIM_IRQN, APP_IRQ_PRIORITY_MID);
    NVIC_EnableIRQ (SPIM_IRQN);
    if(spim_init->rx_done_handler != NULL)
    {
        rx_done = spim_init->rx_done_handler;
    }
    if(spim_init->tx_done_handler != NULL)
    {
        tx_done = spim_init->tx_done_handler;
    }
    SPIM_ID->TXD.LIST = 1;
    SPIM_ID->RXD.LIST = 1;
    mod_is_available = true;
}

void hal_spim_deinit ()
{
    SPIM_ID->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos) &
        SPIM_ENABLE_ENABLE_Msk;
    SPIM_ID->TASKS_SUSPEND = 1;
    SPIM_ID->TASKS_STOP = 1;
    mod_is_available = false;
    
}

void hal_spim_tx_rx (void * p_tx_data, uint32_t tx_len, void * p_rx_data, uint32_t rx_len)
{
    log_printf("%s\n",__func__);
    if(p_tx_data == NULL)
    {
        ASSERT(tx_len == 0);
    }
    if(p_rx_data == NULL)
    {
        ASSERT(rx_len == 0);
    }
    hal_gpio_pin_clear (csBar);

    SPIM_ID->EVENTS_ENDTX = 0;
    SPIM_ID->TXD.PTR = (uint32_t) p_tx_data;
    SPIM_ID->TXD.MAXCNT = tx_len;
    
    SPIM_ID->EVENTS_ENDRX = 0;
    SPIM_ID->RXD.PTR = (uint32_t) p_rx_data;
    SPIM_ID->RXD.MAXCNT = rx_len;

    SPIM_ID->ENABLE = (SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos) &
        SPIM_ENABLE_ENABLE_Msk;
    mod_is_available = false;
    SPIM_ID->TASKS_START = 1;
    (void) SPIM_ID->TASKS_START;
}

uint32_t hal_spim_is_available ()
{
    return (uint32_t)mod_is_available;
}


void SPIM_IRQ_Handler (void)
{
    if(SPIM_ID->EVENTS_END == 1)
    {
        SPIM_ID->EVENTS_END = 0;
        mod_is_available = true;
        hal_gpio_pin_set (csBar);
    }
    if(SPIM_ID->EVENTS_ENDTX == 1 && ((intr_enabled & HAL_SPIM_TX_DONE) != 0))
    {
        SPIM_ID->EVENTS_ENDTX = 0;
        if(tx_done != NULL)
        {
            tx_done(SPIM_ID->TXD.AMOUNT);
        }
    }
    if(SPIM_ID->EVENTS_ENDRX == 1 && ((intr_enabled & HAL_SPIM_RX_DONE) != 0))
    {
        SPIM_ID->EVENTS_ENDRX = 0;
        if(rx_done != NULL)
        {
            rx_done(SPIM_ID->RXD.AMOUNT);
        }
    }
}