/* 
 * File:   aa_aaa_battery_check.h
 * Copyright (c) 2018 Appiko
 * Created on 27 August, 2018, 4:13 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
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
 * @breif Function to get battery status. It'll check if battery % is greater than 20%
 * @return false if battery is below 20%
 */
inline bool aa_aaa_battery_is_charged()
{
    return (simple_adc_get_value(SIMPLE_ADC_GAIN1_6,ANALOG_VDD) >= 2730) ?
        true : false;
}


#ifdef __cplusplus
}
#endif

#endif /* CODEBASE_PERIPHERAL_MODULES_AA_AAA_BATTERY_CHECK_H */

/**
 * @}
 * @}
 */
