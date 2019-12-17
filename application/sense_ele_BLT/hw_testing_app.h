/*
 *  hw_testing_app.h : Tests for SensePi Board level testing
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
 * @addtogroup testing_appln
 * @{
 *
 * @defgroup testing_header Header for hardware testing application
 * @brief This file contains all the test funtion declarations.
 * @{
 *
 */

#ifndef HW_TESTING_APP_H
#define HW_TESTING_APP_H

/**
 * @brief This function is used to check power on the board
 * If program execution starts properly, it means power source is connected
 * @return Value for power test flag
 */
uint32_t power_test(void);
/**
 * @brief This fucntion is used to check DC/DC converter. If DC/DC circuit is
 * present at input supply, funtion will return value 1
 * @return Value for DC/DC test flag 
 */
uint32_t dc_dc_test(void);
/**
 * @brief This fucntion is used to check on low frequency crystal. Without 
 * LF-XTAL we cannot use RTC. Therefore RTC is used to detect LF-XTAL
 * @return Value for Crystal test flag
 */
uint32_t crystal_test(void);

uint32_t accelerometer_test(void);

uint32_t cc1175_clk_test(void);

uint32_t cc1175_transmission_test(void);

uint32_t hall_effect_test(void);

#endif
/**
 * @}
 * @}
 */
