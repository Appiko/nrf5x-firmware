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

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

/** @anchor uart_defines
 * @name Defines for the specific UART peripheral used
 * @{*/
#if defined NRF51
#define UART_ID               CONCAT_2(NRF_UART,UART_USED)
#define UART_IRQN             UART_IRQN_a(UART_USED)
#define UART_IRQ_Handler      UART_IRQ_Handler_a(UART_USED)

#define UART_IRQN_a(n)        UART_IRQN_b(n)
#define UART_IRQN_b(n)        UART##n##_IRQn

#define UART_IRQ_Handler_a(n) UART_IRQ_Handler_b(n)
#define UART_IRQ_Handler_b(n) UART##n##_IRQHandler
/** @} */
#endif
#if defined NRF52832 || defined NRF52810
#define UART_ID               CONCAT_2(NRF_UARTE,UART_USED)
#define UART_IRQN             UART_IRQN_a(UART_USED)
#define UART_IRQ_Handler      UART_IRQ_Handler_a(UART_USED)

#define UART_IRQN_a(n)        UART_IRQN_b(n)
#define UART_IRQN_b(n)        UARTE##n##_UART##n##_IRQn

#define UART_IRQ_Handler_a(n) UART_IRQ_Handler_b(n)
#define UART_IRQ_Handler_b(n) UARTE##n##_UART##n##_IRQHandler
#endif


/** Size of the buffer to hold the received characters before @ref LINE_END is received */
#define BUFFER_SIZE     128

uint8_t tx_buff[1];
uint8_t rx_buff;
/** Buffer to hold the received characters from UART */
uint8_t rx_data[BUFFER_SIZE];

/** Handler to be called when a line of characters is received from UART */
void (*rx_handler)(uint8_t * ptr);

/**
 * @brief Stores the received data in a buffer until @ref LINE_END is received
 *  On receiving @ref LINE_END the configured function pointer is called
 * @param rx_char
 */
static void rx_collect(uint8_t rx_char)
{
    static uint32_t count = 0;
    if (rx_char != LINE_END)
    {
        if (count < BUFFER_SIZE - 1)
        {
            rx_data[count] = rx_char;
            count++;
        }
    }
    else
    {
        rx_data[count] = '\0';
        if (rx_handler != NULL)
        {
            rx_handler(rx_data);
        }
        count = 0;
    }
}

/**
 *  UART interrupt routine.
 *  Only data reception causes interrupt. The received data is passed to @ref rx_collect.
 */
#if ISR_MANAGER == 1
void hal_uart_Handler ()
#else
void UART_IRQ_Handler (void)
#endif
{
#if ISR_MANAGER == 0
    UART_ID->EVENTS_ENDRX = 0;
#endif
    rx_collect(rx_buff);
}

void hal_uart_putchar(uint8_t cr)
{
    tx_buff[0] = cr;
    UART_ID->EVENTS_ENDTX = 0;
    
    while(UART_ID->EVENTS_ENDTX);
    
    UART_ID->TASKS_STARTTX = 1;

    while (!UART_ID->EVENTS_ENDTX)
    {
    }

    UART_ID->EVENTS_ENDTX = 0;
    UART_ID->TASKS_STOPTX = 1;
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
//    (*((uint32_t *)(0x4000250C))) = TX_PIN_NUMBER;
//    (*((uint32_t *)(0x40002514))) = RX_PIN_NUMBER;
    
    UART_ID->PSEL.TXD = TX_PIN_NUMBER;
    UART_ID->PSEL.RXD = RX_PIN_NUMBER;

    
    UART_ID->TXD.PTR = (uint32_t) tx_buff;
    UART_ID->TXD.MAXCNT = 1;

    ///(UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos)
    ///     | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
    UART_ID->CONFIG = 0;

#if HWFC
    {
        /* Configure CTS and RTS pins if HWFC is true in board.h */
        hal_gpio_cfg_output(RTS_PIN_NUMBER, 1);
        hal_gpio_cfg_input(CTS_PIN_NUMBER, HAL_GPIO_PULL_DISABLED);
        (*((uint32_t *)(0x40002508))) = RTS_PIN_NUMBER;
        (*((uint32_t *)(0x40002510))) = CTS_PIN_NUMBER;
        ///(UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos)
        ///    | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
        UART_ID->CONFIG = 1;
    }
#endif

    UART_ID->BAUDRATE = (baud);

    //Enable UART
    UART_ID->ENABLE = (0x08);

    UART_ID->INTENCLR = 0xFFFFFFFF;

    init_printf((void *) !(START_TX), printf_callback);

    if(handler != NULL)
    {
        UART_ID->RXD.PTR = (uint32_t)&rx_buff;
        UART_ID->RXD.MAXCNT = 1;
        UART_ID->EVENTS_ENDRX = 0;

        rx_handler = handler;

        // Enable UART RX interrupt only
        UART_ID->INTENSET = UARTE_INTENSET_ENDRX_Set << UARTE_INTENSET_ENDRX_Pos;

        NVIC_SetPriority(UART_IRQN, APP_IRQ_PRIORITY_LOW);
        NVIC_EnableIRQ(UART_IRQN);

        UART_ID->TASKS_STARTRX = 1;
    }
}
