/*
 *  hal_uart.c
 *
 *  Created on: 23-Jun-2017
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

#include "nrf.h"
#include "hal_uart.h"
#include "boards.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include "stdbool.h"

/** Size of the buffer to hold the received characters before @ref LINE_END is received */
#define BUFFER_SIZE     128

/** Buffer to hold the received characters from UART */
uint8_t rx_buffer[BUFFER_SIZE];

/** Handler to be called when a line of characters is received from UART */
void (*rx_handler)(uint8_t * ptr);

/**
 * @brief Stores the received data in a buffer until @ref LINE_END is received
 *  On receiving @ref LINE_END the configured function pointer is called
 * @param rx_data
 */
static void rx_collect(uint8_t rx_data)
{
    static uint32_t count = 0;
    if (rx_data != LINE_END)
    {
        if (count < BUFFER_SIZE - 1)
        {
            rx_buffer[count] = rx_data;
            count++;
        }
    }
    else
    {
        rx_buffer[count] = '\0';
        if (rx_handler != NULL)
        {
            rx_handler(rx_buffer);
        }
        count = 0;
    }
}

/**
 *  UART interrupt routine.
 *  Only data reception causes interrupt. The received data is passed to @ref rx_collect.
 */
#ifdef NRF51
void UART0_IRQHandler(void)
#endif
#if defined NRF52832 || defined NRF52810
void UARTE0_UART0_IRQHandler(void)
#endif
{
    /* Waits for RX data to be received, but
     * no waiting actually since RX causes interrupt. */
    while (NRF_UARTE0->EVENTS_RXDRDY != 1)
    {
    }
    NRF_UARTE0->EVENTS_RXDRDY = 0;
#if defined NRF52810
    rx_collect((uint8_t) (*((uint32_t *)(0x40002518))));
#else
    rx_collect((uint8_t) NRF_UARTE0->RXD);
#endif
}

void hal_uart_putchar(uint8_t cr)
{
    NRF_UARTE0->TASKS_STARTTX = 1;
#if defined NRF52810
    (*((uint32_t *)(0x4000251C))) = (uint8_t) cr;
#else
    NRF_UARTE0->TXD = (uint8_t) cr;
#endif
    while (NRF_UARTE0->EVENTS_TXDRDY != 1)
    {
    }
    NRF_UARTE0->EVENTS_TXDRDY = 0;
    NRF_UARTE0->TASKS_STOPTX = 1;
}

/**
 * @brief Get the stream of data from the printf formatting module
 * @param str_end A special character sent at the end of a printf string
 * @param ch Individual bytes of data from the string to be sent
 */
void printf_callback(void* str_end, char ch)
{
    hal_uart_putchar(ch);
}

void hal_uart_init(hal_uart_baud_t baud, void (*handler)(uint8_t * ptr))
{
    /* Configure TX and RX pins from board.h */
    hal_gpio_cfg_output(TX_PIN_NUMBER, 1);
    hal_gpio_cfg_input(RX_PIN_NUMBER, HAL_GPIO_PULL_DISABLED);
    NRF_UARTE0->PSEL.TXD = TX_PIN_NUMBER;
    NRF_UARTE0->PSEL.RXD = RX_PIN_NUMBER;

    NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos)
            | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos);

#if HWFC
    {
        /* Configure CTS and RTS pins if HWFC is true in board.h */
        hal_gpio_cfg_output(RTS_PIN_NUMBER, 1);
        hal_gpio_cfg_input(CTS_PIN_NUMBER, HAL_GPIO_PULL_DISABLED);
        NRF_UARTE0->PSEL.RTS = RTS_PIN_NUMBER;
        NRF_UARTE0->PSEL.CTS = CTS_PIN_NUMBER;
        NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos)
                | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos);
    }
#endif

    NRF_UARTE0->BAUDRATE = (baud << UARTE_BAUDRATE_BAUDRATE_Pos);
    NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);
    init_printf((void *) !(START_TX), printf_callback);

    if(handler != NULL)
    {
        NRF_UARTE0->EVENTS_RXDRDY = 0;

        rx_handler = handler;

        // Enable UART RX interrupt only
        NRF_UARTE0->INTENSET = (UARTE_INTENSET_RXDRDY_Set << UARTE_INTENSET_RXDRDY_Pos);

        NVIC_SetPriority(UARTE0_IRQn, APP_IRQ_PRIORITY_LOW);
        NVIC_EnableIRQ(UARTE0_IRQn);

        NRF_UARTE0->TASKS_STARTRX = 1;
    }
}
