/*
 *  hal_pin_analog_input.h
 *
 *  Created on: 28-Mar-2017
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

#ifdef NRF52
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
