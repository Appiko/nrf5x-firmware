/**
 *  irq_msg_util.h : IRQ to main thread message passer
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
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_irq_msg IRQ to main thread message passer
 * @brief This module is used to pass messages from any higher priority
 *  interrupts to the main thread so that the higher
 *  priority interrupt can finish soon and off-load non-real time tasks.
 * @{
 */

#ifndef CODEBASE_UTIL_IRQ_MSG_UTIL_H_
#define CODEBASE_UTIL_IRQ_MSG_UTIL_H_

#include "stdint.h"

typedef enum
{
  MSG_NEXT_INTERVAL,
  MSG_STATE_CHANGE,

  MSG_MAX_SIZE = ((2 ^ 32) - 1) //To make the enum 32 bit long
} irq_msg_types;

typedef struct
{
  void (*next_interval_cb)(uint32_t duration);
  void (*state_change_cb)(uint32_t next_state);
} irq_msg_callbacks;

/**
 * Initialize the messenger ring buffer system
 * @param cb_ptr The array of function pointers that gets called for
 * different message types.
 */
void irq_msg_init(irq_msg_callbacks *cb_ptr);

/**
 * This function is to be called in the higher priority interrupt and
 *  is used to push the required message.
 * @param pushed_msg The type of message to be pushed
 * @param more_data The data of the message to be pushed
 */
void irq_msg_push(irq_msg_types pushed_msg, void *more_data);

/**
 *  This function is to be called in the while(1) loop in main()
 *  so that all the pushed messages can be processed.
 */
void irq_msg_process(void);

#endif /* CODEBASE_UTIL_IRQ_MSG_UTIL_H_ */

/**
 * @}
 * @}
 */
