/**
 *  hal_uart.c : UARTE HAL
 *  Copyright (C) 2020  Appiko
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
#include "hal_uarte.h"
#include "boards.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include "stdbool.h"
#include "string.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

#ifdef NRF52810
#define UART_IRQ_Handler UARTE0_IRQHandler
#define UARTE_IRQN UARTE0_IRQn
#else
#define UART_IRQ_Handler UARTE0_UART0_IRQHandler
#define UARTE_IRQN UARTE0_UART0_IRQn
#endif

/** Size of the buffer to hold the received characters before @ref LINE_END is received
 *   This value must be a power of two to enable increment to 0 */
#define RX_BUFFER_SIZE     8

/** Size of buffer for the transmission of characters from UART and printf string forming
 *  */
#define TX_BUFFER_SIZE     64

/** Buffer to hold the received characters from UART */
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

/** Handler to be called when a characters is received from UART */
void (*rx_handler)(uint8_t rx_byte);

/** Count to check how many more received bytes need to be sent to the handler*/
uint32_t process_count;

/** Count of number of bytes received by UART, incremented in interrupt*/
volatile uint32_t rx_count;

#if ISR_MANAGER == 1
void hal_uart_Handler ()
#else
void UART_IRQ_Handler (void)
#endif
{
    if(NRF_UARTE0->EVENTS_RXDRDY == 1)
    {
        NRF_UARTE0->EVENTS_RXDRDY = 0;
        //This is to increment to zero if it overflows
        rx_count = ((rx_count + 1) & (RX_BUFFER_SIZE - 1));
    }
}

void hal_uarte_start_rx(void (*handler) (uint8_t rx_byte))
{
    rx_handler = handler;

    NRF_UARTE0->EVENTS_RXDRDY = 0;
    NRF_UARTE0->EVENTS_ENDRX = 0;
    NRF_UARTE0->EVENTS_RXTO = 0;
    NRF_UARTE0->EVENTS_RXSTARTED = 0;

    NRF_UARTE0->INTENCLR = 0xFFFFFFFF;
    NRF_UARTE0->INTENSET = (UARTE_INTENSET_RXDRDY_Enabled << UARTE_INTENSET_RXDRDY_Pos);

    NVIC_ClearPendingIRQ(UARTE_IRQN);
    NVIC_EnableIRQ(UARTE_IRQN);

    NRF_UARTE0->RXD.MAXCNT = RX_BUFFER_SIZE/2;
    NRF_UARTE0->RXD.PTR = (uint32_t) (rx_buffer);

    NRF_UARTE0->SHORTS = (UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos);

    rx_count = 0;
    process_count = 0;

    NRF_UARTE0->TASKS_STARTRX = 1;
}

void hal_uarte_stop_rx(void)
{
    NRF_UARTE0->INTENCLR = 0xFFFFFFFF;
    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
    NRF_UARTE0->TASKS_STOPRX = 1;
}

void hal_uarte_putchar(uint8_t cr)
{
    hal_uarte_puts(&cr, 1);
}

void hal_uarte_puts(uint8_t * buff, uint32_t len)
{
    static uint8_t tx_buffer[TX_BUFFER_SIZE];

    if((len == 0) || (buff == NULL))
    {
        return;
    }

    //If the previous transmission is still ongoing
    if((NRF_UARTE0->EVENTS_TXSTARTED == 1) &&
            (NRF_UARTE0->EVENTS_ENDTX == 0))
    {
        while(NRF_UARTE0->EVENTS_ENDTX == 0)
        {
        }
    }

    //Clear the flags
    NRF_UARTE0->EVENTS_TXSTARTED = 0;
    NRF_UARTE0->EVENTS_ENDTX = 0;

    memcpy(tx_buffer, buff, len);

    NRF_UARTE0->TXD.PTR = (uint32_t) tx_buffer;
    NRF_UARTE0->TXD.MAXCNT = len;

    NRF_UARTE0->TASKS_STARTTX = 1;

    //Wait till the UART transmission starts
    while(NRF_UARTE0->EVENTS_TXSTARTED == 0){}
}

/**
 * @brief Get the stream of data from the printf formatting module
 * @param str_end A special character sent at the end of a printf string
 * @param ch Individual bytes of data from the string to be sent
 */
void printf_callback(void* str_end, char ch)
{
    static uint32_t str_count = 0;

    static uint8_t printf_buffer[TX_BUFFER_SIZE];

    hal_gpio_pin_toggle(23);
    if((uint32_t) str_end == START_TX)
    {
        //Since puts is not re-entrant
        CRITICAL_REGION_ENTER();
        hal_uarte_puts(printf_buffer, str_count);
        CRITICAL_REGION_EXIT();
        str_count = 0;
    }
    else
    {
        printf_buffer[str_count] = ch;
        str_count++;
    }
}

void hal_uarte_init(hal_uart_baud_t baud, uint32_t irq_priority)
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
                | (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos)
                | (UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos);
    }
#endif

    NRF_UARTE0->BAUDRATE = (baud);

    NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    NRF_UARTE0->INTENCLR = 0xFFFFFFFF;

    //Initialize the printf if it needs to use this driver
    init_printf((void *) !(START_TX), printf_callback);

    NVIC_SetPriority(UARTE_IRQN, irq_priority);
}

void hal_uarte_uninit(void)
{
    NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos);
    NRF_UARTE0->INTENCLR = 0xFFFFFFFF;
    NVIC_DisableIRQ(UARTE0_UART0_IRQn);
}

void hal_uarte_process(void)
{
    while(process_count != rx_count)
    {
        rx_handler(rx_buffer[process_count]);
        process_count = ((process_count + 1) & (RX_BUFFER_SIZE - 1));

        //To change the DMA double buffered pointer for upcoming data to
        //either first half or second half of the buffer
        if(rx_count == 1)
        {
            NRF_UARTE0->RXD.PTR = (uint32_t) (rx_buffer + (RX_BUFFER_SIZE/2));
        }
        else if(rx_count == ((RX_BUFFER_SIZE/2) + 1))
        {
            NRF_UARTE0->RXD.PTR = (uint32_t) (rx_buffer);
        }
    }
}
