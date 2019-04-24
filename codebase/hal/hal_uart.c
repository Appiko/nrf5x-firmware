/**
 *  hal_uart.c : UART HAL
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
#include "nrf.h"
#include "hal_uart.h"
#include "boards.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include "stdbool.h"

#define UART_IRQn       2

#if defined NRF51
#define UART_PERI   NRF_UART0
#endif
#if defined NRF52832 || defined NRF52810
#define UART_PERI   NRF_UARTE0
#endif


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
#if defined NRF52832
void UARTE0_UART0_IRQHandler(void)
#endif
#if defined NRF52810
void UARTE0_IRQHandler(void)
#endif
{
    /* Waits for RX data to be received, but
     * no waiting actually since RX causes interrupt. */
    while (UART_PERI->EVENTS_RXDRDY != 1)
    {
    }
    UART_PERI->EVENTS_RXDRDY = 0;
    rx_collect((uint8_t) (*((uint32_t *)(0x40002518))));
}

void hal_uart_putchar(uint8_t cr)
{
    (*((uint32_t *)(0x4000251C))) = (uint8_t) cr;
    UART_PERI->EVENTS_TXDRDY = 0;
    UART_PERI->TASKS_STARTTX = 1;

    while (UART_PERI->EVENTS_TXDRDY != 1)
    {
    }

    UART_PERI->EVENTS_TXDRDY = 0;
    UART_PERI->TASKS_STOPTX = 1;
}

/**
 * @brief Get the stream of data from the printf formatting module
 * @param str_end A special character sent at the end of a printf string
 * @param ch Individual bytes of data from the string to be sent
 */
void printf_callback(void* str_end, char ch)
{
    if((uint32_t) str_end != START_TX)
    {
        //Since putchar is not re-entrant
        CRITICAL_REGION_ENTER();
        hal_uart_putchar(ch);
        CRITICAL_REGION_EXIT();
    }
}

void hal_uart_init(hal_uart_baud_t baud, void (*handler)(uint8_t * ptr))
{
    /* Configure TX and RX pins from board.h */
    hal_gpio_cfg_output(TX_PIN_NUMBER, 1);
    hal_gpio_cfg_input(RX_PIN_NUMBER, HAL_GPIO_PULL_DISABLED);
    (*((uint32_t *)(0x4000250C))) = TX_PIN_NUMBER;
    (*((uint32_t *)(0x40002514))) = RX_PIN_NUMBER;

    ///(UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos)
    ///     | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
    UART_PERI->CONFIG = 0;

#if HWFC
    {
        /* Configure CTS and RTS pins if HWFC is true in board.h */
        hal_gpio_cfg_output(RTS_PIN_NUMBER, 1);
        hal_gpio_cfg_input(CTS_PIN_NUMBER, HAL_GPIO_PULL_DISABLED);
        (*((uint32_t *)(0x40002508))) = RTS_PIN_NUMBER;
        (*((uint32_t *)(0x40002510))) = CTS_PIN_NUMBER;
        ///(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos)
        ///    | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
        UART_PERI->CONFIG = 1;
    }
#endif

    UART_PERI->BAUDRATE = (baud);

    //Enable UART
    UART_PERI->ENABLE = (0x04);

    UART_PERI->INTENCLR = 0xFFFFFFFF;

    init_printf((void *) !(START_TX), printf_callback);

    if(handler != NULL)
    {
        UART_PERI->EVENTS_RXDRDY = 0;

        rx_handler = handler;

        // Enable UART RX interrupt only
        UART_PERI->INTENSET = (1 << 2);

        NVIC_SetPriority(UART_IRQn, APP_IRQ_PRIORITY_LOW);
        NVIC_EnableIRQ(UART_IRQn);

        UART_PERI->TASKS_STARTRX = 1;
    }
}
