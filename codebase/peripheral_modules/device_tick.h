/*
 *  device_tick.h
 *
 *  Created on: 08-Mar-2018
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
