/* 
 * File:   sensebe_led_comm.c
 * Copyright (c) 2018 Appiko
 * Created on 14 November, 2018, 5:55 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */

#include "sensebe_led_comm.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "tinyprintf.h"
#include "stdbool.h"
#include "nrf.h"
#include "boards.h"
#include "log.h"

#define UART_IRQn       2

#if defined NRF51
#define UART_PERI   NRF_UART0
#endif
#if defined NRF52832 || defined NRF52810
#define UART_PERI   NRF_UARTE0
#endif

void sensebe_led_comm_init ()
{
    hal_gpio_cfg_output(LED_RED, 1);
    hal_gpio_cfg_input(PHOTODIODE_LIGHT_SENSE, HAL_GPIO_PULL_DISABLED);
    (*((uint32_t *)(0x4000250C))) = LED_RED;
    (*((uint32_t *)(0x40002514))) = PHOTODIODE_LIGHT_SENSE;

    UART_PERI->CONFIG = 0;

    UART_PERI->BAUDRATE = HAL_UART_BAUD_1200;

    //Enable UART
    UART_PERI->ENABLE = (0x04);

    UART_PERI->INTENCLR = 0xFFFFFFFF;

}

void sensebe_led_comm_send (uint8_t * data, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        hal_uart_putchar(data[i]);
    }
    log_init();
    hal_gpio_cfg_output(LED_RED, 0);
}