/**
 *  irq_msg_util.c : IRQ to main thread message passer
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
        case MSG_NEXT_INTERVAL:
            cb_list.next_interval_cb(
                    (uint32_t) (msg.more_data));
            break;
        case MSG_STATE_CHANGE:
            cb_list.state_change_cb(
                    (uint32_t) (msg.more_data));
            break;
        default:
            break;
        }
    }
}
