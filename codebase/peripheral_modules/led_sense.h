/**
 *  led_sense.h : Light sensing using LED
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
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_led_light_sense Light sensing with a LED
 *
 * @brief Light sensing based on sensing the charge accumulated
 *  due to the incident light on a reverse biased LED.
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_LED_SENSE_H_
#define CODEBASE_PERIPHERAL_MODULES_LED_SENSE_H_

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief Initialize the LED light sensing module
 * @param led_out_pin The pin driving the LED
 * @param led_sense_analog_pin The pin at LED's anode to sense light
 * @param led_off_val The digital value at led_out_pin to
 *  switch off the LED.
 */
void led_sense_init(uint32_t led_out_pin,
        uint32_t led_sense_analog_pin, uint32_t led_off_val);

/**
 * @brief Get the light value by measuring the voltage a LED's anode
 * @return The ADC output of the light sensing by the LED. The value
 *  x 3.6/4096 is the actual voltage.
 */
uint32_t led_sense_get(void);

/**
 * @brief Configure the LED as either an light sensing input device
 *  or as an light emitting actuation device
 * @param is_input_on Is input if true and output if false
 */
void led_sense_cfg_input(bool is_input_on);

#endif /* CODEBASE_PERIPHERAL_MODULES_LED_SENSE_H_ */

/**
 * @}
 * @}
 */
