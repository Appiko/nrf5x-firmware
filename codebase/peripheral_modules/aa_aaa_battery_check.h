/**
 *  aa_aaa_battery_check.h : Module to check battery status
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


#ifndef CODEBASE_PERIPHERAL_MODULES_AA_AAA_BATTERY_CHECK_H
#define CODEBASE_PERIPHERAL_MODULES_AA_AAA_BATTERY_CHECK_H

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_battery_check Module to check battery status
 * @brief Simple module to check if battery is greater than 20%.
 * @note While using this module make sure no other module is using SAADC.
 * @{
 */

#include "simple_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function to get battery status. It'll convert battery ADC value to 8bit
 * @return 8bit ADC value corresponding to battery voltage
 */
inline uint8_t aa_aaa_battery_status()
{
    return ((simple_adc_get_value(SIMPLE_ADC_GAIN1_6,ANALOG_VDD) >> 4) & 0xFF);
}


#ifdef __cplusplus
}
#endif

#endif /* CODEBASE_PERIPHERAL_MODULES_AA_AAA_BATTERY_CHECK_H */

/**
 * @}
 * @}
 */
