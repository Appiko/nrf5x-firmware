/*
 *  hal_uart.h
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

#ifndef CODEBASE_HAL_HAL_UART_H_
#define CODEBASE_HAL_HAL_UART_H_

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_uart_hal UART HAL
 * @brief Hardware abstraction layer of the UART peripheral. This is compatible with both
 *  nrf51 and nRF52 SoCs.
 * @{
 */

#include "stdint.h"
#include "stddef.h"

/** The character that is checked by rx_collect to determine if a line of characters is received */
#define LINE_END                '\n'

/**
 * Defines to specify the baudrate options for the uart data transfer
 */
typedef enum {
    HAL_UART_BAUD_1200   = (0x0004F000UL), //!< Uart baud rate of 1200
    HAL_UART_BAUD_2400   = (0x0009D000UL), //!< Uart baud rate of 2400
    HAL_UART_BAUD_4800   = (0x0013B000UL), //!< Uart baud rate of 4800
    HAL_UART_BAUD_9600   = (0x00275000UL), //!< Uart baud rate of 9600
    HAL_UART_BAUD_14400  = (0x003AF000UL), //!< Uart baud rate of 14400
    HAL_UART_BAUD_19200  = (0x004EA000UL), //!< Uart baud rate of 19200
    HAL_UART_BAUD_28800  = (0x0075C000UL), //!< Uart baud rate of 28800
    HAL_UART_BAUD_38400  = (0x009D0000UL), //!< Uart baud rate of 38400
    HAL_UART_BAUD_57600  = (0x00EB0000UL), //!< Uart baud rate of 57600
    HAL_UART_BAUD_76800  = (0x013A9000UL), //!< Uart baud rate of 76800
    HAL_UART_BAUD_115200 = (0x01D60000UL), //!< Uart baud rate of 115200
    HAL_UART_BAUD_230400 = (0x03B00000UL), //!< Uart baud rate of 230400
    HAL_UART_BAUD_250000 = (0x04000000UL), //!< Uart baud rate of 250000
    HAL_UART_BAUD_460800 = (0x07400000UL), //!< Uart baud rate of 460800
    HAL_UART_BAUD_921600 = (0x0F000000UL), //!< Uart baud rate of 921600
    HAL_UART_BAUD_1M     = (0x10000000UL), //!< Uart baud rate of 1M
} hal_uart_baud_t;

/**
 * Function to initialize the parameters of UART based on the configurations in @ref boards.h
 *
 * @param baud The baud rate of operation for the UART module
 * @param handler The handler which is called with a pointer to the data
 *  received on UART reception. If NULL, the UART reception is disabled
 */
void hal_uart_init(hal_uart_baud_t baud, void (*handler) (uint8_t * ptr));

/**
 * @brief Send a single character through UART
 * This function is used by printf_callback so that printf can be used
 * @param cr The character to be sent
 */
void hal_uart_putchar(uint8_t cr);

#endif /* CODEBASE_HAL_HAL_UART_H_ */

/**
 * @}
 * @}
 */
