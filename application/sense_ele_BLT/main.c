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


#define NO_OF_TESTS 7

uint32_t (* tests[]) () = {
    power_test,
    dc_dc_test,
    crystal_test,
    accelerometer_test,
    cc1175_clk_test,
    cc1175_transmission_test,
    hall_effect_test
};

/**
 * @brief This is Entry point for this application and this function will
 * call for every test funtion.
 */

int main(void) {
    hfclk_xtal_init_blocking();
    log_init();

    log_printf("\nSTARTING:SENSE ELE REV 2.0\n");

    hal_nop_delay_ms(250);


    for (int i = 0; i < NO_OF_TESTS; i++) {
        if (!tests[i]()) {
            log_printf("END 0\n");
            return 0;
        }
    }

    log_printf("END 1\n");

    while (1) {
    }
}
