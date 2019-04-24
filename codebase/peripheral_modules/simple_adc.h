/**
 *  simple_adc.h : A simple ADC driver
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

#ifndef CODEBASE_PERIPHERAL_MODULES_SIMPLE_ADC_H_
#define CODEBASE_PERIPHERAL_MODULES_SIMPLE_ADC_H_

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_simple_adc Simple ADC driver
 * @brief A simple driver for the SAADC unit to get ADC values. The spec of this module are:
 *  12 bit resolution, internal reference of 0.6 V, single ended conversion, input impedance
 *  of upto 100k Ohm and a blocking class for the conversion period of 12 us.
 *
 * @{
 */

#include "stdint.h"
#include "nrf.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

#ifndef SAADC_CHANNEL_USED_SIMPLE_ADC 
#define SAADC_CHANNEL_USED_SIMPLE_ADC 1
#endif

/** To assign default SAADC channel to this module if nothing is mentioned  */
#ifndef SIMPLE_ADC_CHANNEL_USED 
#define SIMPLE_ADC_CHANNEL_USED    SAADC_CHANNEL_USED_SIMPLE_ADC
#endif
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
