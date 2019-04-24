/**
 *  device_tick.h : Device Tick generator
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

#ifndef CODEBASE_PERIPHERAL_MODULES_DEVICE_TICK_H_
#define CODEBASE_PERIPHERAL_MODULES_DEVICE_TICK_H_

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_device_tick Device ticker
 *
 * @brief Driver for the device tick generator. This module is responsible for
 *  generating the next interval events at configurable fast or slow intervals.
 *
 * @warning This module needs the LFCLK and @ref group_ms_timer to be on and
 *  running to be able to work. Also the irq_msg_util module needs to be initialized.
 * @{
 */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef MS_TIMER_USED_DEVICE_TICKS 
#define MS_TIMER_USED_DEVICE_TICKS 0
#endif

/** @brief The number of MS timer ticks elapsed for one device tick */
#define DEVICE_TICK_MSTIMER_DIV_FACTOR        (1)

/**
 * @brief The two different modes at which the device ticks, fast & slow
 */
typedef enum
{
    DEVICE_TICK_SLOW, //!< Slow mode
    DEVICE_TICK_FAST, //!< Fast mode
    DEVICE_TICK_SAME  //!< Same as previous mode
}device_tick_mode;

/**
 * @brief Stucture for passing the configuration for initializing the
 *  Device Tick module.
 */
typedef struct {
  uint32_t fast_mode_ticks;   /// The number of LFCLK ticks for the fast mode
  uint32_t slow_mode_ticks;   /// The number of LFCLK ticks for the slow mode
  device_tick_mode mode;      /// The initial mode in which the module is in
}device_tick_cfg;

/**
 * @brief Initializes and starts the Device tick module. Provides a next
 *  interval event immediately. This function can also be used to reinitialize
 *  the fast and slow mode intervals.
 * @param cfg The initial mode as well as the fast and slow interval
 *  in number of LFCLK ticks.
 */
void device_tick_init(device_tick_cfg *cfg);

/**
 * @brief Switches from the fast to slow mode or vice versa.
 * @param mode The mode to switch to.
 */
void device_tick_switch_mode(device_tick_mode mode);

/**
 * @brief The function to be called whenever the SoC wakes up to see if more
 *  than half the time of the current interval has elapsed to call the next
 *  interval handler.
 */
void device_tick_process(void);

#endif /* CODEBASE_PERIPHERAL_MODULES_DEVICE_TICK_H_ */
/**
 * @}
 * @}
 */
