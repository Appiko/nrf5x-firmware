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
 * @defgroup bluey_demo An application to demo the Bluey's capabilities
 * @brief A Bluey demo
 * @{
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

#include "boards.h"
#include "hal_clocks.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "common_util.h"
#include "tinyprintf.h"
#include "uart_printf.h"
#include "us_timer.h"
#include "nrf_util.h"
#include "ble_adv.h"
#include "ms_timer.h"
#include "profiler_timer.h"

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
    hal_gpio_pin_write(LED_RED, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

static void log_dump_handler(void){
    dump_log();
    static uint8_t count = 0;
    count++;
    uint8_t adv_data[] = {
            /* Len, type, data */
            0x02, GAP_ADV_FLAGS, 0x04,
            0x04, GAP_ADV_NAME_FULL,'E','f','H',
            0x02, 0xFF, count
    };
    ble_adv_set_adv_data(sizeof(adv_data), adv_data);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    uart_printf_init(UART_PRINTF_BAUD_1M);
    tfp_printf("Hello World %d!\n", 42);

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    us_timer_init(APP_IRQ_PRIORITY_LOWEST);
    profiler_timer_init();

    hfclk_xtal_init_blocking();
    lfclk_init(LFCLK_SRC_Xtal);

    uint8_t adrs[] = {0x0B, 0x0E, 0x0A, 0x0C, 0x00, 0x01};
    uint8_t adv_data[] = {
            /* Len, type, data */
            0x02, GAP_ADV_FLAGS, 0x04,
            0x04, GAP_ADV_NAME_FULL,'E','f','H',
            0x02, GAP_ADV_MANUF_DATA, 0
    };

    uint8_t scan_rsp[] = {
            0x02, GAP_ADV_TRANSMIT_PWR, 0
    };

PROFILE_START;
    ble_adv_param_t param = {ADV_INTERVAL_MS(500), ADV_SCAN_IND_PARAM, RANDOM_ADRS_PARAM, CH_ALL_PARAM};

    ble_adv_set_random_adrs(adrs);
    ble_adv_set_tx_power(0);
    ble_adv_set_adv_data(sizeof(adv_data), adv_data);
    ble_adv_set_adv_param(&param);
    ble_adv_set_scan_rsp_data(sizeof(scan_rsp), scan_rsp);

    ble_adv_start();

    ms_timer_start(MS_TIMER0, MS_REPEATED_CALL, RTC_TICKS_MS(1027), log_dump_handler);
PROFILE_STOP;

    while (true)
    {
        __WFI();
    }
}

/** @} */
/** @} */
