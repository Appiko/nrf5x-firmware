/*
 *  simple_adc.h
 *
 *  Created on: 15-May-2017
 *
 *  Copyright (c) 2017, Appiko
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

#ifndef CODEBASE_PERIPHERAL_MODULES_SIMPLE_ADC_H_
#define CODEBASE_PERIPHERAL_MODULES_SIMPLE_ADC_H_

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_simple_adc Simple ADC driver
 * @brief A simple driver for the SAADC unit to get ADC values. The spec of this module are:
 *  12 bit resolution, internal reference of 0.4 V, single ended conversion, input impedance
 *  of upto 100k Ohm and a blocking class for the conversion period of 12 us.
 *
 * @{
 */

#include "stdint.h"
#include "nrf.h"

/**
 * @brief Input selection for the analog-to-digital converter.
 */
typedef enum
{
    ANALOG_PIN_2  = SAADC_CH_PSELP_PSELP_AnalogInput0,
    ANALOG_PIN_3  = SAADC_CH_PSELP_PSELP_AnalogInput1,
    ANALOG_PIN_4  = SAADC_CH_PSELP_PSELP_AnalogInput2,
    ANALOG_PIN_5  = SAADC_CH_PSELP_PSELP_AnalogInput3,
    ANALOG_PIN_28 = SAADC_CH_PSELP_PSELP_AnalogInput4,
    ANALOG_PIN_29 = SAADC_CH_PSELP_PSELP_AnalogInput5,
    ANALOG_PIN_30 = SAADC_CH_PSELP_PSELP_AnalogInput6,
    ANALOG_PIN_31 = SAADC_CH_PSELP_PSELP_AnalogInput7,
    ANALOG_VDD       = SAADC_CH_PSELP_PSELP_VDD
} simple_adc_input_t;

/**
 * @brief Gain factor of the ADC input source.
 */
typedef enum
{
    SIMPLE_ADC_GAIN1_6 = SAADC_CH_CONFIG_GAIN_Gain1_6, ///< Gain factor 1/6.
    SIMPLE_ADC_GAIN1_5 = SAADC_CH_CONFIG_GAIN_Gain1_5, ///< Gain factor 1/5.
    SIMPLE_ADC_GAIN1_4 = SAADC_CH_CONFIG_GAIN_Gain1_4, ///< Gain factor 1/4.
    SIMPLE_ADC_GAIN1_3 = SAADC_CH_CONFIG_GAIN_Gain1_3, ///< Gain factor 1/3.
    SIMPLE_ADC_GAIN1_2 = SAADC_CH_CONFIG_GAIN_Gain1_2, ///< Gain factor 1/2.
    SIMPLE_ADC_GAIN1   = SAADC_CH_CONFIG_GAIN_Gain1,   ///< Gain factor 1.
    SIMPLE_ADC_GAIN2   = SAADC_CH_CONFIG_GAIN_Gain2,   ///< Gain factor 2.
    SIMPLE_ADC_GAIN4   = SAADC_CH_CONFIG_GAIN_Gain4,   ///< Gain factor 4.
} simple_adc_gain_t;

/**
 * @brief This function initializes the SAADC peripheral, gets an ADC value and then deinitializes
 *  The function blocks for about 13 us for the ADC sampling to occur.
 * @param gain The gain applied to the analog input to the ADC unit
 * @param pin The pin at which the analog sample needs to be converted to digital
 * @warning There are only a few pins that can be used with the ADC peripheral,
 *  check in the datasheet
 * @return The converted digital value of the analog signal
 */
uint32_t simple_adc_get_value(simple_adc_gain_t gain, simple_adc_input_t pin);

#endif /* CODEBASE_PERIPHERAL_MODULES_SIMPLE_ADC_H_ */

/**
 * @}
 * @}
 */
