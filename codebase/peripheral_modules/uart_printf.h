/*
 *  uart_printf.h
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

#ifndef CODEBASE_PERIPHERAL_MODULES_UART_PRINTF_H_
#define CODEBASE_PERIPHERAL_MODULES_UART_PRINTF_H_

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_uart_printf UART printf
 * @brief UART Driver for sending printf messages.
 *  To be used in conjunction with @ref tinyprintf.h
 *
 * @{
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * Defines to specify the baudrate options for the printf messages
 */
typedef enum {
    UART_PRINTF_BAUD_1200 = UART_BAUDRATE_BAUDRATE_Baud1200,    //!< UART_PRINTF_BAUD_1200
    UART_PRINTF_BAUD_2400 = UART_BAUDRATE_BAUDRATE_Baud2400,    //!< UART_PRINTF_BAUD_2400
    UART_PRINTF_BAUD_4800 = UART_BAUDRATE_BAUDRATE_Baud4800,    //!< UART_PRINTF_BAUD_4800
    UART_PRINTF_BAUD_9600 = UART_BAUDRATE_BAUDRATE_Baud9600,    //!< UART_PRINTF_BAUD_9600
    UART_PRINTF_BAUD_14400 = UART_BAUDRATE_BAUDRATE_Baud14400,  //!< UART_PRINTF_BAUD_14400
    UART_PRINTF_BAUD_19200 = UART_BAUDRATE_BAUDRATE_Baud19200,  //!< UART_PRINTF_BAUD_19200
    UART_PRINTF_BAUD_28800 = UART_BAUDRATE_BAUDRATE_Baud28800,  //!< UART_PRINTF_BAUD_28800
    UART_PRINTF_BAUD_38400 = UART_BAUDRATE_BAUDRATE_Baud38400,  //!< UART_PRINTF_BAUD_38400
    UART_PRINTF_BAUD_56000 = UART_BAUDRATE_BAUDRATE_Baud56000,  //!< UART_PRINTF_BAUD_56000
    UART_PRINTF_BAUD_57600 = UART_BAUDRATE_BAUDRATE_Baud57600,  //!< UART_PRINTF_BAUD_57600
    UART_PRINTF_BAUD_76800 = UART_BAUDRATE_BAUDRATE_Baud76800,  //!< UART_PRINTF_BAUD_76800
    UART_PRINTF_BAUD_115200 = UART_BAUDRATE_BAUDRATE_Baud115200,//!< UART_PRINTF_BAUD_115200
    UART_PRINTF_BAUD_230400 = UART_BAUDRATE_BAUDRATE_Baud230400,//!< UART_PRINTF_BAUD_230400
    UART_PRINTF_BAUD_250000 = UART_BAUDRATE_BAUDRATE_Baud250000,//!< UART_PRINTF_BAUD_250000
    UART_PRINTF_BAUD_460800 = UART_BAUDRATE_BAUDRATE_Baud460800,//!< UART_PRINTF_BAUD_460800
    UART_PRINTF_BAUD_921600 = UART_BAUDRATE_BAUDRATE_Baud921600,//!< UART_PRINTF_BAUD_921600
    UART_PRINTF_BAUD_1M = UART_BAUDRATE_BAUDRATE_Baud1M,        //!< UART_PRINTF_BAUD_1M
} uart_printf_baud_t;

/**
 * @brief Function to initialize the parameters of UART based on the
 *  configurations in @ref boards.h
 * @param baud_rate The baud rate used for the UART to send the printf messages
 */
void uart_printf_init(uart_printf_baud_t baud_rate);

#endif /* CODEBASE_PERIPHERAL_MODULES_UART_PRINTF_H_ */

/**
 * @}
 * @}
 */
