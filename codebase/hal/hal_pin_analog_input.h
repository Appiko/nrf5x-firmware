/**
 *  hal_pin_analog_input.h : Analog Input HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CODEBASE_HAL_HAL_PIN_ANALOG_INPUT_H_
#define CODEBASE_HAL_HAL_PIN_ANALOG_INPUT_H_

/** @brief Macro to convert the pin number to its respective analog input number.
 * If the particular pin does not have an analog input, a macro undeclared error
 * is shown at compile time.
 * @param PIN_NUM The pin number whose analog input number is needed
 */
#define PIN_TO_ANALOG_INPUT(PIN_NUM)    PIN_TO_ANALOG_INPUT_(PIN_NUM)
/** Private macro used by @ref PIN_TO_ANALOG_INPUT */
#define PIN_TO_ANALOG_INPUT_(PIN_NUM)    __ANALOG_PIN_OF_##PIN_NUM

#ifdef NRF51
#error Check these defines, not yet done properly yet
/** @warning This pin is also the connection to 32 kHz crystal */
#define __ANALOG_PIN_OF_26                0
/** @warning This pin is also the connection to 32 kHz crystal */
#define __ANALOG_PIN_OF_27                1
#define __ANALOG_PIN_OF_1                 2
#define __ANALOG_PIN_OF_2                 3
#define __ANALOG_PIN_OF_3                 4
#define __ANALOG_PIN_OF_4                 5
#define __ANALOG_PIN_OF_5                 6
/** @warning This pin is also the AREF1 input */
#define __ANALOG_PIN_OF_6                 7
#endif

#if defined NRF52832 || defined NRF52810
#define __ANALOG_PIN_OF_2                 SAADC_CH_PSELP_PSELP_AnalogInput0
#define __ANALOG_PIN_OF_3                 SAADC_CH_PSELP_PSELP_AnalogInput1
#define __ANALOG_PIN_OF_4                 SAADC_CH_PSELP_PSELP_AnalogInput2
#define __ANALOG_PIN_OF_5                 SAADC_CH_PSELP_PSELP_AnalogInput3
#define __ANALOG_PIN_OF_28                SAADC_CH_PSELP_PSELP_AnalogInput4
#define __ANALOG_PIN_OF_29                SAADC_CH_PSELP_PSELP_AnalogInput5
#define __ANALOG_PIN_OF_30                SAADC_CH_PSELP_PSELP_AnalogInput6
#define __ANALOG_PIN_OF_31                SAADC_CH_PSELP_PSELP_AnalogInput7
#endif
#endif /* CODEBASE_HAL_HAL_PIN_ANALOG_INPUT_H_ */
