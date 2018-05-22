/*
 *  led_sense.h
 *
 *  Created on: 04-May-2018
 *
 *  Copyright (c) 2018, Appiko
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
