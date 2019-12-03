/**
 *  hal_spim.c : SPI Master HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stddef.h>

#include "hal_spim.h"
#include "common_util.h"
#include "hal_gpio.h"
#include "stddef.h"
#include "nrf_assert.h"
#include "log.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

/** @anchor twim_defines
 * @name Defines for the specific SPI peripheral used 
 * @{*/
#define SPIM_ID CONCAT_2(NRF_SPIM,SPIM_USED)


#define SPIM_IRQ_Handler SPIM_IRQ_Handler_a(SPIM_USED)
#define SPIM_IRQ_Handler_a(n) SPIM_IRQ_Handler_b(n)
#ifdef NRF52832
#define SPIM_IRQ_Handler_b(n) SPIM##n##_SPIS##n##_TWIM##n##_TWIS##n##_SPI##n##_TWI##n##_IRQHandler
#endif
#ifdef NRF52810
#define SPIM_IRQ_Handler_b(n) SPIM##n##_SPIS##n##_IRQHandler
#endif

#define SPIM_IRQN SPIM_IRQN_a(SPIM_USED)
#define SPIM_IRQN_a(n)  SPIM_IRQN_b(n)

#ifdef NRF52840
#define SPIM_IRQN_b(n)  SPIM##n##_SPIS##n##_TWIM##n##_TWIS##n##_SPI##n##_TWI##n##_IRQn
#endif
#ifdef NRF52810
#define SPIM_IRQN_b(n)  SPIM##n##_SPIS##n##_IRQn
#endif
#ifdef NRF52832
#define SPIM_IRQN_b(n)  SPIM##n##_SPIS##n##_TWIM##n##_TWIS##n##_SPI##n##_TWI##n##_IRQn
#endif
/** @} */

/** variable to store CS Bar pin number */
static uint32_t csBar = 0;

/** Variable to store interrupts which are to be enabled */
static uint32_t intr_enabled = 0;

/** Status flag */
static volatile bool mod_is_busy = false;

/** Function pointer buffers */
void (*rx_done) (uint32_t last_byte_no);
void (*tx_done) (uint32_t last_byte_no);

void hal_spim_init (hal_spim_init_t * spim_init)
{
    hal_gpio_cfg_output (spim_init->csBar_pin, 1);
    SPIM_ID->TASKS_SUSPEND = 1;
    SPIM_ID->TASKS_STOP = 1;
    SPIM_ID->CONFIG = spim_init->byte_order | ((spim_init->spi_mode)<<1);
    SPIM_ID->FREQUENCY = spim_init->freq;
    SPIM_ID->PSEL.MISO = spim_init->miso_pin;
    SPIM_ID->PSEL.MOSI = spim_init->mosi_pin;
    SPIM_ID->PSEL.SCK = spim_init->sck_pin;
    csBar = spim_init->csBar_pin;
    intr_enabled = spim_init->en_intr;
    SPIM_ID->INTENSET = spim_init->en_intr | SPIM_INTENSET_END_Msk;
        NVIC_SetPriority (SPIM_IRQN, APP_IRQ_PRIORITY_MID);
        NVIC_EnableIRQ (SPIM_IRQN);
        log_printf("Intr En : %d\n", intr_enabled);
    if(intr_enabled != 0)
    {
        NVIC_SetPriority (SPIM_IRQN, spim_init->irq_priority);
        NVIC_EnableIRQ (SPIM_IRQN);
    }
    else
    {
    }
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
    mod_is_busy = false;
}

void hal_spim_deinit ()
{
    SPIM_ID->ENABLE = (SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos) &
        SPIM_ENABLE_ENABLE_Msk;
    SPIM_ID->INTENCLR = intr_enabled;
    SPIM_ID->TASKS_SUSPEND = 1;
    SPIM_ID->TASKS_STOP = 1;
    mod_is_busy = true;
    NVIC_DisableIRQ (SPIM_IRQN);    
}

void hal_spim_tx_rx (void * p_tx_data, uint32_t tx_len, void * p_rx_data, uint32_t rx_len)
{
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
    mod_is_busy = true;
    SPIM_ID->TASKS_START = 1;
    (void) SPIM_ID->TASKS_START;
}

uint32_t hal_spim_is_busy ()
{
    return (uint32_t)mod_is_busy;
}

#if ISR_MANAGER == 1
void hal_spim_Handler (void)
#else
void SPIM_IRQ_Handler (void)
#endif
{
    if(SPIM_ID->EVENTS_END == 1)
    {
#if ISR_MANAGER == 0
        SPIM_ID->EVENTS_END = 0;
#endif
        mod_is_busy = false;
        hal_gpio_pin_set (csBar);
    }
    if(SPIM_ID->EVENTS_ENDTX == 1 && ((intr_enabled & HAL_SPIM_TX_DONE) != 0))
    {
#if ISR_MANAGER == 0
        SPIM_ID->EVENTS_ENDTX = 0;
#endif
        if(tx_done != NULL)
        {
            tx_done(SPIM_ID->TXD.AMOUNT);
        }
    }
    if(SPIM_ID->EVENTS_ENDRX == 1 && ((intr_enabled & HAL_SPIM_RX_DONE) != 0))
    {
#if ISR_MANAGER == 0
        SPIM_ID->EVENTS_ENDRX = 0;
#endif
        if(rx_done != NULL)
        {
            rx_done(SPIM_ID->RXD.AMOUNT);
        }
    }
}