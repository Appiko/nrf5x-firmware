/*
 *  clocks.h
 *
 *  Created on: 06-Feb-2017
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

#ifndef CODEBASE_HAL_HAL_CLOCKS_H_
#define CODEBASE_HAL_HAL_CLOCKS_H_

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_clocks Clocks HAL
 * @brief Hardware abstraction layer of the high frequency and low frequency clock.
 * @{
 */

#include "stdbool.h"
#include "nrf.h"

/** The possible sources for Low frequency clock's 32 kHz input
 */
typedef enum {
    LFCLK_SRC_RC    = CLOCK_LFCLKSRC_SRC_RC,   ///< Internal RC oscillator.
    LFCLK_SRC_Xtal  = CLOCK_LFCLKSRC_SRC_Xtal, ///< External crystal.
    LFCLK_SRC_Synth = CLOCK_LFCLKSRC_SRC_Synth ///< Synthesizer from HFCLK clock.
} lfclk_src_t;

///Frequency of the low frequency clock is 32.768 kHz
#define LFCLK_FREQ 32768

/** @brief Function to initialize the LF clock
 * @param lfclk_src The source for the lf clock (RC, Xtal or systhesis from HF-clk)
 * @warning If the clock source is RC oscillator, calibrate it to use (errata 77 in nRF52)
 */
void lfclk_init(lfclk_src_t lfclk_src);

/** @brief Function to de-initialize the LF clock.
 * Saves a bit of power as LF clock is not running, but RTC or WDT cannot run
 * @warning Might be better to leave it on as it consumes little power.
 * @warning Be aware of errata 132 in nRF52 (LF RC Osc doesn't restart in some cases)
 */
void lfclk_deinit(void);

/** @brief Function to start the crystal oscillator to be used for HF clock.
 *  This function blocks until the crystal oscillator starts.
 * @warning Beware of errata 68 in nRF52 */
void hfclk_xtal_init_blocking(void);

/** @brief Function to initialize the HF clock to use the crystal.
 *  This function returns immediately after triggering the start task. */
void hfclk_xtal_init_nonblocking(void);

/** @brief Blocks until the HF crystal oscillator is running
 * @warning The HF crystal oscillator must be started before this call.
 * @warning Beware of errata 68 in nRF52
 */
void hfclk_block_till_xtal(void);

/** @brief Function to de-initialize the HF clock from using the crystal.
 * RC oscillator will be used to generate HF clock. More power and not accurate.
 * Starts quick though.*/
void hfclk_xtal_deinit(void);


#endif /* CODEBASE_HAL_HAL_CLOCKS_H_ */

/**
 * @}
 * @}
 */
