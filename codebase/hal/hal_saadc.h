/*
 *  saadc.h
 *
 *  Created on: 15-Jan-2017
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

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_saadc SAADC HAL
 * @brief Hardware abstraction layer of the Successive Approximation Analog to Digital
 *  Converter (SAADC) peripheral.
 * @note This peripheral is present only in nRF52 SoCs.
 * @{
 */

#ifndef CODEBASE_HAL_HAL_SAADC_H_
#define CODEBASE_HAL_HAL_SAADC_H_

#include "nrf_saadc.h"
#include "nrf_assert.h"
#include "hal_pin_analog_input.h"

#ifdef NRF51
#error SAADC peripheral is not present in the nRF51 SoC
#endif

/**
 * @brief To be consistent with the saadc naming for the function here.
 *  The function definition is in the SDK HAL.
 */
#define saadc_channel_init      nrf_saadc_channel_init

/**
 * @brief Function for configuring the use of the internal sample-rate
 *       timer and its interval
 *
 * @param[in] sampling_rate_div The sampling rate is (16/sampling_rate_div) MHz
 */
__STATIC_INLINE void saadc_continuous_sampling_rate_set(uint32_t sampling_rate_div)
{
    ASSERT((sampling_rate_div>=80) && (sampling_rate_div <= 2047));

    NRF_SAADC->SAMPLERATE = (sampling_rate_div << SAADC_SAMPLERATE_CC_Pos) |
            (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos);
}

/**
 * @brief Function that sets the sampling to be started with SAMPLING task
 */
__STATIC_INLINE void saadc_sampling_task_mode_set(void)
{
    NRF_SAADC->SAMPLERATE = (0 << SAADC_SAMPLERATE_CC_Pos) |
            (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
}

/**
 * @brief Function to uninitializes a SAADC channel.
 */
__STATIC_INLINE void saadc_channel_uninit(uint8_t channel)
{
    nrf_saadc_channel_input_set(channel, NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
}


#endif /* CODEBASE_HAL_HAL_SAADC_H_ */

/**
 * @}
 * @}
 */
