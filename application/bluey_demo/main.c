/*  Copyright (c) 2016, Appiko
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

/**
 * @addtogroup group_appln
 * @{
 *
 * @defgroup bluey_demo A application to demo the Bluey's capabilities
 * @brief A Bluey demo
 * @{
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

#include "boards.h"
#include "hal_clocks.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "nrf_assert.h"
#include "ms_timer.h"
#include "nrf_util.h"
#include "nrf_delay.h"
#include "common_util.h"
#include "tinyprintf.h"
#include "uart_printf.h"

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
    nrf_gpio_cfg_output(LED_RED);
    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_cfg_output(LED_BLUE);

    nrf_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
    nrf_gpio_pin_write(LED_RED, (LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    nrf_delay_ms(250);
    nrf_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    nrf_delay_ms(250);
    nrf_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
    nrf_delay_ms(250);
    nrf_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    nrf_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

void repeated_handler(void){
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    uart_printf_init(UARTE_BAUDRATE_BAUDRATE_Baud1M);
    tfp_printf("Hello World %ld!\n", 1);

    hfclk_xtal_init_blocking();
    lfclk_init(BOARD_LFCLKSRC);

    while (true)
    {
        __WFI();
    }
}

/** @} */
/** @} */
