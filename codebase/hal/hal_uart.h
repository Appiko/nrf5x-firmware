/**
 *  hal_uart.h : UART HAL
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
