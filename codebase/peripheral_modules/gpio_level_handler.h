/*
 *  gpio_level_handler.h
 *
 *  Created on: 30-Mar-2018
 *
 *  Copyright (c) 2018, Appiko
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
 * @defgroup group_gpio_level GPIO level handler
 * @brief Driver to use GPIOTE port event to detect whenever any of multiple GPIO pins
 *  are at a specified polarity. This module can be used to wake the nRF SoC from the
 *  SYSTEM OFF mode and consumes about 1 uA of current.
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_GPIO_LEVEL_HANDLER_H_
#define CODEBASE_PERIPHERAL_MODULES_GPIO_LEVEL_HANDLER_H_

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief The configuration parameter for each of the GPIO pin
 *  that is being configured in the GPIO level handler module
 */
typedef struct
{
    void (*handler)(bool is_still_on);  ///The handler that's called when
                             ///this pin is at the specified level
    uint32_t pin_num;       ///The pin number
    uint32_t pull_cfg;      ///The pull up/down or disabled config as per
                             ///the nrf bitfields header file
    bool trigger_on_high;   ///A boolean to indicate if the module should
                             ///trigger on high or low level for this pin
}gpio_level_cfg;

/**
 * @brief Initializes the GPIO level handler module by taking in the the
 *  configurations of all the GPIOs that need to be monitored
 * @param cfg A pointer to an array of GPIO configurations which need to be
 *  configured to generate port events and call the appropriate handler
 * @param cfg_num The size of the array
 * @param irq_priority The priority of the GPIOTE port event IRQ handler
 */
void gpio_level_init(gpio_level_cfg * cfg, uint32_t cfg_num, uint32_t irq_priority);

#endif /* CODEBASE_PERIPHERAL_MODULES_GPIO_LEVEL_HANDLER_H_ */

/**
 * @}
 * @}
 */
