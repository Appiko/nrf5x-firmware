/*
 *  simple_pwm.h
 *
 *  Created on: 19-Jun-2017
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

/** Specify which Timer peripheral would be used for the simple PWM module */
#define SIMPLE_PWM_TIMER_USED           1

///The number of CC registers in the RTC peripheral used for MS timer
#define SIMPLE_PWM_CC_COUNT           CONCAT_3(TIMER, SIMPLE_PWM_TIMER_USED, _CC_NUM)

/// Three GPIOTE channels are used from this number for this module
#define SIMPLE_PWM_GPIOTE_START_CH    0

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
}simple_pwm_channels;

/**
 * @brief The format of the PWM configuration data
 */
typedef struct
{
    uint32_t pin[SIMPLE_PWM_MAX_CHANNEL]; ///Specify's the three pins
    simple_pwm_timer_freq_t freq;         ///Freq of timer running the PWM
    ///Max count of the PWM timer, implies the resolution
    uint32_t max_count;
}simple_pwm_config;

/**
 * @brief Initializes the simple PWM module
 * @param config The PWM configuration data
 */
void simple_pwm_init(simple_pwm_config * config);

/**
 * @brief Set a new value to a PWM channel
 * @param channel The channel whose PWM value is updated
 * @param value The new PWM value
 */
void simple_pwm_set_value(simple_pwm_channels channel, uint32_t value);

#endif /* CODEBASE_PERIPHERAL_MODULES_SIMPLE_PWM_H_ */
