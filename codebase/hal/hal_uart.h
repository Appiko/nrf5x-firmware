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

/** The character that is checked by @ref rx_collect to determine if a line of characters is received */
#define LINE_END                '\n'

/**
 * Defines to specify the baudrate options for the uart data transfer
 */
typedef enum {
    HAL_UART_BAUD_1200 = UART_BAUDRATE_BAUDRATE_Baud1200,    //!< Uart baud rate of 1200
    HAL_UART_BAUD_2400 = UART_BAUDRATE_BAUDRATE_Baud2400,    //!< Uart baud rate of 2400
    HAL_UART_BAUD_4800 = UART_BAUDRATE_BAUDRATE_Baud4800,    //!< Uart baud rate of 4800
    HAL_UART_BAUD_9600 = UART_BAUDRATE_BAUDRATE_Baud9600,    //!< Uart baud rate of 9600
    HAL_UART_BAUD_14400 = UART_BAUDRATE_BAUDRATE_Baud14400,  //!< Uart baud rate of 14400
    HAL_UART_BAUD_19200 = UART_BAUDRATE_BAUDRATE_Baud19200,  //!< Uart baud rate of 19200
    HAL_UART_BAUD_28800 = UART_BAUDRATE_BAUDRATE_Baud28800,  //!< Uart baud rate of 28800
    HAL_UART_BAUD_38400 = UART_BAUDRATE_BAUDRATE_Baud38400,  //!< Uart baud rate of 38400
    HAL_UART_BAUD_56000 = UART_BAUDRATE_BAUDRATE_Baud56000,  //!< Uart baud rate of 56000
    HAL_UART_BAUD_57600 = UART_BAUDRATE_BAUDRATE_Baud57600,  //!< Uart baud rate of 57600
    HAL_UART_BAUD_76800 = UART_BAUDRATE_BAUDRATE_Baud76800,  //!< Uart baud rate of 76800
    HAL_UART_BAUD_115200 = UART_BAUDRATE_BAUDRATE_Baud115200,//!< Uart baud rate of 115200
    HAL_UART_BAUD_230400 = UART_BAUDRATE_BAUDRATE_Baud230400,//!< Uart baud rate of 230400
    HAL_UART_BAUD_250000 = UART_BAUDRATE_BAUDRATE_Baud250000,//!< Uart baud rate of 250000
    HAL_UART_BAUD_460800 = UART_BAUDRATE_BAUDRATE_Baud460800,//!< Uart baud rate of 460800
    HAL_UART_BAUD_921600 = UART_BAUDRATE_BAUDRATE_Baud921600,//!< Uart baud rate of 921600
    HAL_UART_BAUD_1M = UART_BAUDRATE_BAUDRATE_Baud1M,        //!< Uart baud rate of 1M
} hal_uart_baud_t;

/**
 * Function to initialize the parameters of UART based on the configurations in @ref board.h
 *
 * @param baud The baud rate of operation for the UART module
 * @param handler The handler which is called with a pointer to the data
 *  received on UART reception
 */
void hal_uart_init(hal_uart_baud_t baud, void (*handler) (uint8_t * ptr));

/**
 * @brief Send a single character through UART
 * This function is used by @ref printf_callback so that printf can be used
 * @param cr The character to be sent
 */
void hal_uart_putchar(uint8_t cr);

#endif /* CODEBASE_HAL_HAL_UART_H_ */

/**
 * @}
 * @}
 */
