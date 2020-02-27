/**
 *  hal_uarte.h : UARTE HAL
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

#ifndef CODEBASE_HAL_HAL_UARTE_H_
#define CODEBASE_HAL_HAL_UARTE_H_

#ifdef CODEBASE_HAL_HAL_UART_H_
#error "UART is being used by older driver"
#endif

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_uarte_hal UARTE HAL
 * @brief Hardware abstraction layer of the UARTE peripheral. This peripheral
 *  uses EASY DMA of UART and is compatible with NRF52 SoCs
 * @{
 */

#include "stdint.h"
#include "stddef.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

#include "nrf.h"

/**
 * Defines to specify the baudrate options for the uart data transfer
 */
typedef enum {
    HAL_UARTE_BAUD_1200   = (UARTE_BAUDRATE_BAUDRATE_Baud1200), //!< Uart baud rate of 1200
    HAL_UARTE_BAUD_2400   = (UARTE_BAUDRATE_BAUDRATE_Baud2400), //!< Uart baud rate of 2400
    HAL_UARTE_BAUD_4800   = (UARTE_BAUDRATE_BAUDRATE_Baud4800), //!< Uart baud rate of 4800
    HAL_UARTE_BAUD_9600   = (UARTE_BAUDRATE_BAUDRATE_Baud9600), //!< Uart baud rate of 9600
    HAL_UARTE_BAUD_14400  = (UARTE_BAUDRATE_BAUDRATE_Baud14400), //!< Uart baud rate of 14400
    HAL_UARTE_BAUD_19200  = (UARTE_BAUDRATE_BAUDRATE_Baud19200), //!< Uart baud rate of 19200
    HAL_UARTE_BAUD_28800  = (UARTE_BAUDRATE_BAUDRATE_Baud28800), //!< Uart baud rate of 28800
    HAL_UARTE_BAUD_38400  = (UARTE_BAUDRATE_BAUDRATE_Baud38400), //!< Uart baud rate of 38400
    HAL_UARTE_BAUD_57600  = (UARTE_BAUDRATE_BAUDRATE_Baud57600), //!< Uart baud rate of 57600
    HAL_UARTE_BAUD_76800  = (UARTE_BAUDRATE_BAUDRATE_Baud76800), //!< Uart baud rate of 76800
    HAL_UARTE_BAUD_115200 = (UARTE_BAUDRATE_BAUDRATE_Baud115200), //!< Uart baud rate of 115200
    HAL_UARTE_BAUD_230400 = (UARTE_BAUDRATE_BAUDRATE_Baud230400), //!< Uart baud rate of 230400
    HAL_UARTE_BAUD_250000 = (UARTE_BAUDRATE_BAUDRATE_Baud250000), //!< Uart baud rate of 250000
    HAL_UARTE_BAUD_460800 = (UARTE_BAUDRATE_BAUDRATE_Baud460800), //!< Uart baud rate of 460800
    HAL_UARTE_BAUD_921600 = (UARTE_BAUDRATE_BAUDRATE_Baud921600), //!< Uart baud rate of 921600
    HAL_UARTE_BAUD_1M     = (UARTE_BAUDRATE_BAUDRATE_Baud1M), //!< Uart baud rate of 1M
} hal_uart_baud_t;

/**
 * Function to initialize the parameters of UARTE based on the configurations in @ref boards.h
 *
 * @param baud The baud rate of operation for the UART module
 * @param irq_priority Priority of the interrupts during UART reception
 */
void hal_uarte_init(hal_uart_baud_t baud, uint32_t irq_priority);

/**
 * Function to uninitialize and disable the UARTE peripheral
 */
void hal_uarte_uninit(void);

/**
 * @brief Send a single character through UART
 * This function can be used by printf_callback so that printf can be used
 * @param cr The character to be sent
 */
void hal_uarte_putchar(uint8_t cr);

/**
 * @brief Send an array of characters through UART
 * This function can be used by printf_callback so that printf can be used
 * @param buff The pointer to the buffer containing the characters to be sent
 * @param len Length of buffer to be sent
 */
void hal_uarte_puts(uint8_t * buff, uint32_t len);
/**
 * @brief Starts the reception of data on UART
 * @param handler The handler which is called with the byte received
 */
void hal_uarte_start_rx(void (*handler) (uint8_t rx_byte));

/**
 * @brief Stops the reception of data on UART
 */
void hal_uarte_stop_rx(void);

/**
 *  This function is to be called in the while(1) loop in main()
 *  so that all bytes received by UART is sent to a handler
 */
void hal_uarte_process(void);

#endif /* CODEBASE_HAL_HAL_UART_H_ */

/**
 * @}
 * @}
 */
