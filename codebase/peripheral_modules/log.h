/**
 *  log.h : output method selections
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

#ifndef CODEBASE_PERIPHERAL_MODULES_LOG_H_
#define CODEBASE_PERIPHERAL_MODULES_LOG_H_

#if defined LOG_BMP_SWO
#define log_printf(...)
#elif defined LOG_SEGGER_RTT
#include "SEGGER_RTT.h"
#define log_init()
#define log_printf(...)  SEGGER_RTT_printf(0, __VA_ARGS__)
#elif defined LOG_UART_DMA_PRINTF//UARTE printf
#include "nrf.h"
#include "tinyprintf.h"
#include "uart_printf.h"
#define log_init()       uart_printf_init(UART_PRINTF_BAUD_1M)
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic push
#define log_printf(...)  tfp_printf(__VA_ARGS__)
#pragma GCC diagnostic pop
#elif defined LOG_UART_PRINTF//UART printf
#include "nrf.h"
#include "tinyprintf.h"
#include "hal_uart.h"
#define log_init()       hal_uart_init(HAL_UART_BAUD_1M, NULL)
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic push
#define log_printf(...)  tfp_printf(__VA_ARGS__)
#pragma GCC diagnostic pop
#elif defined LOG_GPS//UART printf
#include "nrf.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include "hal_uarte.h"
#define log_init()       hal_uarte_init(HAL_UARTE_BAUD_9600, APP_IRQ_PRIORITY_LOW)
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic push
#define log_printf(...)  tfp_printf(__VA_ARGS__)
#pragma GCC diagnostic pop
#else
#define log_init()
#define log_printf(...)
#endif

#endif /* CODEBASE_PERIPHERAL_MODULES_LOG_H_ */
