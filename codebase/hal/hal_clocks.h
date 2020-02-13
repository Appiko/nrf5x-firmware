/**
 *  hal_clocks.h : Clocks HAL
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

/**
 * @brief Function to get source of HF-CLK
 * @return HFCLK Source
 * @retval 0 : RC
 * @retval 1 : XTal
 */
uint32_t hfclk_xtal_get_src ();


#endif /* CODEBASE_HAL_HAL_CLOCKS_H_ */

/**
 * @}
 * @}
 */
