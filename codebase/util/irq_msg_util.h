/*
 *  irq_msg_util.h
 *
 *  Created on: 30-Jan-2018
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
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_irq_msg IRQ to main thread message passer
 * @brief This module is used to pass messages from any higher priority
 *  interrupts to lower priority ones or the main thread so that the higher
 *  priority interrupt can finish soon and off-load non-real time tasks.
 * @{
 */

#ifndef CODEBASE_UTIL_IRQ_MSG_UTIL_H_
#define CODEBASE_UTIL_IRQ_MSG_UTIL_H_

#include "stdint.h"

typedef enum {
  MSG_NEXT_INTERVAL,
  MSG_STATE_CHANGE,

  MSG_MAX_SIZE = ((2^32)-1)    //To make the enum 32 bit long
}irq_msg_types;

typedef struct {
  void (*next_interval_cb)(uint32_t duration);
  void (*state_change_cb)(uint32_t next_state);
}irq_msg_callbacks;

/**
 * Initialize the messenger ring buffer system
 * @param cb_ptr The array of function pointers that gets called for
 * different message types.
 */
void irq_msg_init(irq_msg_callbacks * cb_ptr);

/**
 * This function is to be called in the higher priority interrupt and
 *  is used to push the required message.
 * @param pushed_msg The type of message to be pushed
 * @param more_data The data of the message to be pushed
 */
void irq_msg_push(irq_msg_types pushed_msg, void * more_data);

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
