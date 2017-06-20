/*
 *  uart_printf.c
 *
 *  Created on: 27-Apr-2017
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
#include "uart_printf.h"
#include "hal_gpio.h"
#include "boards.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include <stdbool.h>

/** Size of the PING & PONG buffer each to hold the data to be sent*/
#define BUFFER_SIZE     128

typedef enum
{
    PING,
    PONG
} uart_buffers;

typedef enum
{
    PING_DONE,
    PING_TX,
    PONG_DONE,
    PONG_TX,
} uart_tx_states;

typedef enum
{
    EMPTY,
    FILLING,
    FILLED
} uart_buffer_states;

static const struct buffer_state
{
    uart_tx_states DONE;
    uart_tx_states TX;
} BUF_STATE[2] = {{ PING_DONE, PING_TX },
                  { PONG_DONE, PONG_TX } };

struct uart_tx_buffer
{
    uint8_t __attribute__((aligned(32))) buf[BUFFER_SIZE];
    uint32_t count;
    uart_buffer_states state;
};

static struct uart_context
{
    volatile struct uart_tx_buffer tx_buf[2];
    volatile uart_tx_states tx_state;
} uart_ctx = { { { { 0 }, 0, EMPTY }, { { 0 }, 0, EMPTY } }, PING_DONE };

static void stop_uart(void)
{
    NRF_UARTE0->TASKS_STOPTX = 1;
}

static void start_uart_tx(uart_buffers tx_buf)
{
    NRF_UARTE0->EVENTS_ENDTX = 0;

    NRF_UARTE0->TXD.PTR = (uint32_t) uart_ctx.tx_buf[tx_buf].buf;
    NRF_UARTE0->TXD.MAXCNT = uart_ctx.tx_buf[tx_buf].count;

    NRF_UARTE0->TASKS_STARTTX = 1;
}

/**
 *  UARTE interrupt routine.
 */
void UARTE0_UART0_IRQHandler(void)
{
    if (1 == NRF_UARTE0->EVENTS_ENDTX)
    {
        NRF_UARTE0->EVENTS_ENDTX = 0;
        (void) NRF_UARTE0->EVENTS_ENDTX;

        uart_buffers current_buf = PONG, other_buf = PING;
        if (BUF_STATE[PING].TX == uart_ctx.tx_state)
        {
            current_buf = PING;
            other_buf = PONG;
        }   // Else state of PONG_TX is taken care of in initialization

        uart_ctx.tx_buf[current_buf].state = EMPTY;
        uart_ctx.tx_buf[current_buf].count = 0;

        if (FILLED == uart_ctx.tx_buf[other_buf].state)
        {
            start_uart_tx(other_buf);
            uart_ctx.tx_state = BUF_STATE[other_buf].TX;
        }
        else
        {
            uart_ctx.tx_state = BUF_STATE[current_buf].DONE;
            stop_uart();    //So that UARTE doesn't consume power
        }
    }
}

/**
 */
void printf_callback(void* str_end, char ch)
{
    uart_buffers current_buf = PONG, other_buf = PING;
    if ((BUF_STATE[PING].TX == uart_ctx.tx_state) || (BUF_STATE[PING].DONE == uart_ctx.tx_state))
    {
        current_buf = PING;
        other_buf = PONG;
    }

    //Callback informing the end of the printf string
    if (START_TX == ((uint32_t) str_end))
    {
        if (PING_DONE == uart_ctx.tx_state)
        {
            start_uart_tx(PONG);
            uart_ctx.tx_state = PONG_TX;
        }
        if (PONG_DONE == uart_ctx.tx_state)
        {
            start_uart_tx(PING);
            uart_ctx.tx_state = PING_TX;
        }
        uart_ctx.tx_buf[other_buf].state = FILLED;

    }
    else
    {
        // Check for exceeding the buffer size
        if (uart_ctx.tx_buf[other_buf].count < BUFFER_SIZE)
        {
            uart_ctx.tx_buf[other_buf].buf[uart_ctx.tx_buf[other_buf].count] = ch;
            uart_ctx.tx_buf[other_buf].count++;
            uart_ctx.tx_buf[other_buf].state = FILLING;
        }
        else
        {
            while (BUF_STATE[current_buf].TX == uart_ctx.tx_state)
            {
                //__WFI();
            }
            start_uart_tx(other_buf);
            uart_ctx.tx_buf[other_buf].state = FILLED;    // Not required, but just for completeness
            uart_ctx.tx_state = BUF_STATE[other_buf].TX;
        }
    }
}

void uart_printf_init(uart_printf_baud_t baud_rate)
{
    /* Configure TX and RX pins from board.h */
    hal_gpio_cfg_output(TX_PIN_NUMBER, 0);
    hal_gpio_cfg_input(RX_PIN_NUMBER, GPIO_PIN_CNF_PULL_Disabled);
    NRF_UARTE0->PSEL.TXD = TX_PIN_NUMBER;
    NRF_UARTE0->PSEL.RXD = RX_PIN_NUMBER;

    /* Configure CTS and RTS pins if HWFC is true in board.h */
    if (HWFC)
    {
        hal_gpio_cfg_output(RTS_PIN_NUMBER, 0);
        hal_gpio_cfg_input(CTS_PIN_NUMBER, GPIO_PIN_CNF_PULL_Disabled);
        NRF_UARTE0->PSEL.RTS = RTS_PIN_NUMBER;
        NRF_UARTE0->PSEL.CTS = CTS_PIN_NUMBER;
        NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Enabled << UARTE_CONFIG_HWFC_Pos);
    }

    /* Configure other UART parameters, BAUD rate is defined in nrf52-uart.h    */
    NRF_UARTE0->BAUDRATE = (baud_rate << UARTE_BAUDRATE_BAUDRATE_Pos);
    NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos);

    //Send in any other value other than START_TX in first argument
    init_printf((void *) !(START_TX), printf_callback);

    // Enable UART TX End interrupt only
    NRF_UARTE0->INTENSET = (UARTE_INTENSET_ENDTX_Set << UARTE_INTENSET_ENDTX_Pos);

    NVIC_SetPriority(UARTE0_UART0_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(UARTE0_UART0_IRQn);

}
/** @} */
