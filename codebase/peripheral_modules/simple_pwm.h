/**
 *  simple_pwm.h : A simple PWM driver
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

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_simple_pwm Simple PWM driver
 * @brief A simple driver to get three PWM channels. This PWM module uses three
 *  GPIOTE channels and six PPI channels
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_SIMPLE_PWM_H_
#define CODEBASE_PERIPHERAL_MODULES_SIMPLE_PWM_H_

#include "stdint.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef TIMER_USED_SIMPLE_PWM 
#define TIMER_USED_SIMPLE_PWM 1
#endif

/** Specify which Timer peripheral would be used for the simple PWM module */
#define SIMPLE_PWM_TIMER_USED           TIMER_USED_SIMPLE_PWM

///The number of CC registers in the RTC peripheral used for MS timer
#define SIMPLE_PWM_CC_COUNT           CONCAT_3(TIMER, SIMPLE_PWM_TIMER_USED, _CC_NUM)

/// Three GPIOTE channels are used from this number for this module
#define SIMPLE_PWM_GPIOTE_START_CH    0

#define SIMPLE_PWM_PPI_CHS_USED 6

/**
 * @brief Defines for the frequency at which the timer should run
 *  for the PWM generation
 */
typedef enum
{
    SIMPLE_PWM_TIMER_FREQ_16MHz = 0, ///< PWM timer frequency of 16 MHz.
    SIMPLE_PWM_TIMER_FREQ_8MHz,      ///< PWM timer frequency of 8 MHz.
    SIMPLE_PWM_TIMER_FREQ_4MHz,      ///< PWM timer frequency of 4 MHz.
    SIMPLE_PWM_TIMER_FREQ_2MHz,      ///< PWM timer frequency of 2 MHz.
    SIMPLE_PWM_TIMER_FREQ_1MHz,      ///< PWM timer frequency of 1 MHz.
    SIMPLE_PWM_TIMER_FREQ_500kHz,    ///< PWM timer frequency of 500 kHz.
    SIMPLE_PWM_TIMER_FREQ_250kHz,    ///< PWM timer frequency of 250 kHz.
    SIMPLE_PWM_TIMER_FREQ_125kHz,    ///< PWM timer frequency of 125 kHz.
    SIMPLE_PWM_TIMER_FREQ_62500Hz,   ///< PWM timer frequency of 62500 Hz.
    SIMPLE_PWM_TIMER_FREQ_31250Hz    ///< PWM timer frequency of 31250 Hz.
} simple_pwm_timer_freq_t;

/**
 * @brief Defines for specifying the three PWM channels
 */
typedef enum
{
    SIMPLE_PWM_CHANNEL0,//!< SIMPLE_PWM_CHANNEL0
    SIMPLE_PWM_CHANNEL1,//!< SIMPLE_PWM_CHANNEL1
    SIMPLE_PWM_CHANNEL2,//!< SIMPLE_PWM_CHANNEL2
    SIMPLE_PWM_MAX_CHANNEL
}simple_pwm_channel_t;

/**
 * @brief Function to initiate PWM module with clock frequency and total time of pulse
 * @param freq Frequency at which driving clock works.
 * @param max_count Max number of pulses possible. i.e. total time in ticks
 */
void simple_pwm_init (simple_pwm_timer_freq_t freq,uint32_t max_count);

/**
 * @brief Function to setup a channel
 * @param channel Channel which is to be set.
 * @param pwm_out_pin Output GPIO pin where PWM signal is desired
 * @param value Number of pulses. i.e. on time in ticks
 */
void simple_pwm_channel_setup(simple_pwm_channel_t channel, uint32_t pwm_out_pin, 
                                uint32_t value);

/**
 * @brief Function to start PWM signals
 */
void simple_pwm_start ();

/**
 * @brief Function to stop PWM signals
 */
void simple_pwm_stop ();

#endif /* CODEBASE_PERIPHERAL_MODULES_SIMPLE_PWM_H_ */
/**
 * @}
 * @}
 */
