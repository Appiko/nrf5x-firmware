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
 * @brief This fucntion is used to check on board LEDs. Operator has to be there
 * to check if LEDs are blinking properly or not. 
 * @return Value for LED test flag
 */
uint32_t led_test(void);
/**
 * @brief This fucntion is used to check on low frequency crystal. Without 
 * LF-XTAL we cannot use RTC. Therefore RTC is used to detect LF-XTAL
 * @return Value for Crystal test flag
 */
uint32_t crystal_test(void);
/**
 * @brief This fucntion is used to check output RC very low frequency filter.
 * This function will actully check for any AC part present in offset voltage.
 * @return Value for RC filter test flag
 */
uint32_t rc_test(void);
/**
 * @brief This fucntion is used to check output filter designed for f(max)=10Hz
 * This test will detect if any other frequency is supplied to device other
 * than 10 Hz
 * @return Value for Frequency filter test flag
 */
uint32_t freq_filter_test(void);
/**
 * @brief This fucntion is used to check MCP4012 pot. In this function we will
 * check an ratio between two signals obtained by changing gain by changing pot
 * values. 
 * @return Value for POT test flag
 */
uint32_t pot_test(void);
#endif
/**
 * @}
 * @}
 */
