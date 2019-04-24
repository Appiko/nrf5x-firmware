/**
 *  gpio_level_handler.h : GPIO Level Handler Driver
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
