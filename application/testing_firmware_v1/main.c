/*
 *  main.c : Application for board level testing of SensePi devices
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

/**
 * @addtogroup group_appln
 * @{
 *
 * @defgroup testing_appln Hardware testing application for Sense Rev4 board
 * @brief The main file for hardware testing application which is used to test 
 * functioning of board.
 * @{
 *
 */

#include <stdint.h>
#include "nrf.h"
#include "boards.h"

#include "hal_clocks.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"

#include "mcp4012_x.h"
#include "log.h"

#include "hw_testing_app.h"
#include "product_id.h"

#define NO_OF_TESTS 7

static uint32_t flag_rev_pol_pro = 0;
static uint32_t flag_led = 0;
static uint32_t flag_dc_dc = 0;
static uint32_t flag_pot = 0;
static uint32_t flag_rc_output = 0;
static uint32_t flag_crystal = 0;
static uint32_t flag_freq_filter = 0;
/**
 * @brief This is Entry point for this application and this function will
 * call for every test funtion.
 */

int main(void)
{
    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
    hfclk_xtal_init_blocking();
    log_init();
    log_printf("\nHARDWARE TEST\n");
    log_printf("START\n");
    mcp4012_init(MCP4012T_CS_PIN,MCP4012T_UD_PIN,SPI_SCK_PIN);
    hal_nop_delay_ms(250);
    flag_rev_pol_pro = power_test();
    flag_dc_dc     = dc_dc_test();
    flag_led     = led_test();
    flag_crystal     = crystal_test(); 
    flag_rc_output     = rc_test();
    flag_freq_filter = freq_filter_test();
    flag_pot     = pot_test();
    if( flag_rev_pol_pro && flag_dc_dc && flag_led && flag_pot && flag_rc_output
     && flag_crystal && flag_freq_filter )
    {
        hal_gpio_pin_clear(LED_RED);
        hal_gpio_pin_set(LED_GREEN);
        log_printf("Status : 1\n");
        log_printf("END\n");
    }
    else
    {
        hal_gpio_pin_clear(LED_GREEN);
        hal_gpio_pin_set(LED_RED);
        log_printf("Status : 0\n");
        log_printf("END\n");
    }
    while(1)
    {
    }
}
/**
 * @}
 * @}
 */
