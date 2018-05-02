/*
 *  main.c
 *
 *  Created on: 4-Apr-2018
 *
 *  Copyright (c) 2018, Appiko
 *  Author : Tejas Vasekar (https://github.com/tejas-tj)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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

    hal_nop_delay_ms(10000);

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
