/**
 *  uart_printf.h : UART Driver for sending printf messages
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
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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

#ifndef UARTE_BAUDRATE_BAUDRATE_Pos
#error "UARTE (with Easy DMA) peripheral isn't available in this SoC. \
    Use the UART peripheral."
#endif

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef UARTE_USED_UART_PRINTF
#define UARTE_USED_UART_PRINTF 0
#endif

/**
 * Defines to specify the baudrate options for the printf messages
 */
typedef enum {
    UART_PRINTF_BAUD_1200 = UARTE_BAUDRATE_BAUDRATE_Baud1200,    //!< UART_PRINTF_BAUD_1200
    UART_PRINTF_BAUD_2400 = UARTE_BAUDRATE_BAUDRATE_Baud2400,    //!< UART_PRINTF_BAUD_2400
    UART_PRINTF_BAUD_4800 = UARTE_BAUDRATE_BAUDRATE_Baud4800,    //!< UART_PRINTF_BAUD_4800
    UART_PRINTF_BAUD_9600 = UARTE_BAUDRATE_BAUDRATE_Baud9600,    //!< UART_PRINTF_BAUD_9600
    UART_PRINTF_BAUD_14400 = UARTE_BAUDRATE_BAUDRATE_Baud14400,  //!< UART_PRINTF_BAUD_14400
    UART_PRINTF_BAUD_19200 = UARTE_BAUDRATE_BAUDRATE_Baud19200,  //!< UART_PRINTF_BAUD_19200
    UART_PRINTF_BAUD_28800 = UARTE_BAUDRATE_BAUDRATE_Baud28800,  //!< UART_PRINTF_BAUD_28800
    UART_PRINTF_BAUD_38400 = UARTE_BAUDRATE_BAUDRATE_Baud38400,  //!< UART_PRINTF_BAUD_38400
    UART_PRINTF_BAUD_57600 = UARTE_BAUDRATE_BAUDRATE_Baud57600,  //!< UART_PRINTF_BAUD_57600
    UART_PRINTF_BAUD_76800 = UARTE_BAUDRATE_BAUDRATE_Baud76800,  //!< UART_PRINTF_BAUD_76800
    UART_PRINTF_BAUD_115200 = UARTE_BAUDRATE_BAUDRATE_Baud115200,//!< UART_PRINTF_BAUD_115200
    UART_PRINTF_BAUD_230400 = UARTE_BAUDRATE_BAUDRATE_Baud230400,//!< UART_PRINTF_BAUD_230400
    UART_PRINTF_BAUD_250000 = UARTE_BAUDRATE_BAUDRATE_Baud250000,//!< UART_PRINTF_BAUD_250000
    UART_PRINTF_BAUD_460800 = UARTE_BAUDRATE_BAUDRATE_Baud460800,//!< UART_PRINTF_BAUD_460800
    UART_PRINTF_BAUD_921600 = UARTE_BAUDRATE_BAUDRATE_Baud921600,//!< UART_PRINTF_BAUD_921600
    UART_PRINTF_BAUD_1M = UARTE_BAUDRATE_BAUDRATE_Baud1M,        //!< UART_PRINTF_BAUD_1M
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
