/*
 *  irq_msg_util.c
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

#include "irq_msg_util.h"
#include "stdbool.h"
#include "stddef.h"
#include "CBUF.h"
#include "nrf_util.h"
#include "string.h"
#include "nrf_assert.h"

//For all the code not public for file, message is shortened to msg

/** @brief Size of the ring buffer for the message list
 *  @note Should be a power of 2 */
#define msgbuf_SIZE  16

/** Check if MSG_SIZE is power of 2 */
#if (!(!(msgbuf_SIZE & (msgbuf_SIZE-1)) && msgbuf_SIZE))
#error msgbuf_SIZE must be a power of 2
#endif

struct message
{
    irq_msg_types type;
    void * more_data;
};

volatile static struct
{
    uint32_t m_getIdx;
    uint32_t m_putIdx;
    struct message m_entry[msgbuf_SIZE];
} msgbuf;

irq_msg_callbacks cb_list = { NULL, NULL };

void irq_msg_init(irq_msg_callbacks * cb_ptr)
{
    CBUF_Init(msgbuf);

    ASSERT((cb_ptr->next_interval_cb != NULL)
            && (cb_ptr->state_change_cb != NULL));

    cb_list.next_interval_cb = cb_ptr->next_interval_cb;
    cb_list.state_change_cb = cb_ptr->state_change_cb;
}

void irq_msg_push(irq_msg_types pushed_msg, void * more_data)
{
    CRITICAL_REGION_ENTER();

    struct message msg;
    msg.type = pushed_msg;
    msg.more_data = more_data;

    CBUF_Push(msgbuf, msg);

    CRITICAL_REGION_EXIT();
}

void irq_msg_process(void)
{
    while (CBUF_Len(msgbuf))
    {
        struct message msg = CBUF_Pop(msgbuf);

//    log_printf("_%x\n",msg.type);
        switch (msg.type)
        {
        case NEXT_INTERVAL:
            cb_list.next_interval_cb(
                    (uint32_t) (msg.more_data));
            break;
        case STATE_CHANGE:
            cb_list.state_change_cb(
                    (uint32_t) (msg.more_data));
            break;
        default:
            break;
        }
    }
}
