/**
 *  hal_saadc.h : SAADC HAL
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
