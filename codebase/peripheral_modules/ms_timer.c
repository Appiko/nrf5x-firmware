/* 	Copyright (c) 2017, Appiko
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without modification,
 *	are permitted provided that the following conditions are met:
 *
 *	1. Redistributions of source code must retain the above copyright notice,
 *	this list of conditions and the following disclaimer.
 *
 *	2. Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the documentation
 *	and/or other materials provided with the distribution.
 *
 *	3. Neither the name of the copyright holder nor the names of its contributors
 *	may be used to endorse or promote products derived from this software without
 *	specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *	IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *	OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *	POSSIBILITY OF SUCH DAMAGE.
 */

#include "ms_timer.h"
#include "stddef.h"

/** @anchor rtc_defines
 * @name Defines for the specific RTC peripheral used for ms timer
 * @{*/
#define RTC_ID              CONCAT_2(NRF_RTC,MS_TIMER_RTC_USED)
#define RTC_IRQN            CONCAT_3(RTC, MS_TIMER_RTC_USED, _IRQn)
#define RTC_IRQ_Handler     CONCAT_3(RTC, MS_TIMER_RTC_USED, _IRQHandler)
/** @} */

/**
 * Structure to hold the @ref timer_mode and handler to be called on expiration
 */
static struct ms_timer_t
{
    volatile uint32_t timer_mode;
    void (*timer_handler)(void);
} ms_timer[MS_TIMER_MAX];

/** Timers currently used based on the first four bits from LSB */
static volatile uint32_t ms_timers_status;

void ms_timer_init(uint32_t irq_priority)
{
    RTC_ID->TASKS_STOP = 1;

    for (ms_timer_num i = MS_TIMER0; i < MS_TIMER_MAX; i++)
    {
        ms_timer[i].timer_mode = MS_SINGLE_CALL;
        ms_timer[i].timer_handler = NULL;
    }

    ms_timers_status = 0;
    RTC_ID->PRESCALER = (LFCLK_FREQ/MS_TIMER_FREQ) - 1;

    NVIC_SetPriority(RTC_IRQN, irq_priority);
    NVIC_EnableIRQ(RTC_IRQN);
}

/**@todo Take care of values of ticks passed which are greater than 2^24, now it is
 *  suppressed to 2^24 when greater.
 * @warning Works only for input less than 512000 milli-seconds or 8.5 min when
 *  @ref MS_TIMER_FREQ is 32768
 */
void ms_timer_start(ms_timer_num id, ms_timer_mode mode, uint32_t ticks, void (*handler)(void))
{

    /* make sure the number of ticks to interrupt is less than 2^24 */
    ticks = (ticks & RTC_COUNTER_COUNTER_Msk);

    ms_timer[id].timer_handler = handler;
    if (mode == MS_SINGLE_CALL)
    {
        ms_timer[id].timer_mode = MS_SINGLE_CALL;
    }
    else
    {
        ms_timer[id].timer_mode = ticks;
    }

    RTC_ID->CC[id] = RTC_ID->COUNTER + ticks;

    RTC_ID->EVENTS_COMPARE[id] = 0;
    RTC_ID->EVTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
    RTC_ID->INTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);

    if (ms_timers_status == 0)
    {
        RTC_ID->TASKS_START = 1;
    }
    ms_timers_status |= 1 << id;
}

void ms_timer_stop(ms_timer_num id)
{
    ms_timer[id].timer_mode = MS_SINGLE_CALL;
    ms_timers_status &= ~(1 << id);
    RTC_ID->EVTENCLR = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
    RTC_ID->INTENCLR = 1 << (RTC_INTENSET_COMPARE0_Pos + id);

    if (ms_timers_status == 0)
    {
        RTC_ID->TASKS_STOP = 1;
    }
}

bool ms_timer_get_on_status(ms_timer_num id)
{
    return ((ms_timers_status & (1 << id)) != 0);
}

/** @brief Function for handling the RTC interrupts.
 * Triggered Compare register of timer ID
 */
__attribute__((optimize("unroll-loops")))
void RTC_IRQ_Handler()
{
    for (ms_timer_num id = MS_TIMER0; id < MS_TIMER_MAX; id++)
    {
        if (RTC_ID->EVENTS_COMPARE[id])
        {
            RTC_ID->EVENTS_COMPARE[id] = 0;
            (void)RTC_ID->EVENTS_COMPARE[id];

            void (*cb_handler)(void) = NULL;
            if (ms_timer[id].timer_handler != NULL)
            {
                cb_handler = ms_timer[id].timer_handler;
            }

            if (ms_timer[id].timer_mode == MS_SINGLE_CALL)
            {
                ms_timer_stop(id);
            }
            else
            {
                RTC_ID->CC[id] += ms_timer[id].timer_mode;
            }

            if(cb_handler != NULL)
            {
                cb_handler();
            }
        }
    }
}
/**
 * @}
 */
