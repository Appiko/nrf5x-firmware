/**
 *  ms_timer.c : Millisecond timer
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

#include "ms_timer.h"
#include "stddef.h"
#include "nrf_assert.h"

#if ISR_MANAGER == 1
#include "template_isr_manage.h"
#endif

/** @anchor rtc_defines
 * @name Defines for the specific RTC peripheral used for ms timer
 * @{*/
#define RTC_ID              CONCAT_2(NRF_RTC,MS_TIMER_RTC_USED)
#define RTC_IRQN            CONCAT_3(RTC, MS_TIMER_RTC_USED, _IRQn)
#define RTC_IRQ_Handler     CONCAT_3(RTC, MS_TIMER_RTC_USED, _IRQHandler)
/** @} */


/** Maximum value which can be counted with 24 bit RTC counter */
#define RTC_MAX_COUNT 0xFFFFFF

/**
 * Structure to hold the @ref timer_mode and handler to be called on expiration
 */
static struct ms_timer_t
{
    volatile uint64_t timer_mode;
    volatile uint32_t timer_over_flow_num;
    void (*timer_handler)(void);
} ms_timer[MS_TIMER_MAX];

/** Timers currently used based on the first four bits from LSB */
static volatile uint32_t ms_timers_status;

/** To keep track of which timer needs an overflow event */
static volatile uint32_t overflow_req_status;

/**
 * @brief Function to check number of overflow required and ticks after overflows are done 
 * @param counter_val Current RTC counter value.
 * @param ticks Ticks for which MS_TIMER has been started 
 * @param id MS_TIMER_ID which is to be started
 * @return Number of ticks after number of overflows are done
 */
static void cal_overflow_ticks_req (uint32_t counter_val, uint64_t ticks, uint32_t id)
{
    //Check if ticks can counted before overflow happens
    if(ticks < RTC_MAX_COUNT)
    {
        
        ms_timer[id].timer_over_flow_num = 0;
        RTC_ID->CC[id] = (ticks + counter_val) & (0xFFFFFF);
        RTC_ID->EVTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
        RTC_ID->INTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
        overflow_req_status &= 0<<id;
    }
    else
    {
        ms_timer[id].timer_over_flow_num = (ticks - 
                        (RTC_MAX_COUNT - counter_val)) / (1<<24) + 1;
        RTC_ID->CC[id] = (ticks - (RTC_MAX_COUNT - counter_val)) % (0x1 << 24);
        overflow_req_status |= 1<<id;
        RTC_ID->INTENSET = RTC_INTENSET_OVRFLW_Msk;
        RTC_ID->EVTENSET = RTC_EVTENSET_OVRFLW_Msk;
    }
}

/**
 * @brief Function to handle RTC_EVENT_OVRFLW
 */
void rtc_overflow_handler ()
{
    for (ms_timer_num id = MS_TIMER0; id < MS_TIMER_MAX; id++)
    {
        if(ms_timer[id].timer_over_flow_num != 0)
        {
            if(ms_timer[id].timer_over_flow_num == 1)
            {
                RTC_ID->EVTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
                RTC_ID->INTENSET = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
                overflow_req_status &= 0<<id;
            }
            ms_timer[id].timer_over_flow_num--;
        }
    }
    if(overflow_req_status == 0)
    {
        RTC_ID->INTENCLR = RTC_INTENSET_OVRFLW_Msk;
        RTC_ID->EVTENCLR = RTC_EVTENSET_OVRFLW_Msk;
    }
    
}

void ms_timer_init(uint32_t irq_priority)
{
    RTC_ID->TASKS_STOP = 1;

    for (ms_timer_num i = MS_TIMER0; i < MS_TIMER_MAX; i++)
    {
        ms_timer[i].timer_mode = MS_SINGLE_CALL;
        ms_timer[i].timer_over_flow_num = -1;
        ms_timer[i].timer_handler = NULL;
    }

    ms_timers_status = 0;
    RTC_ID->PRESCALER = (ROUNDED_DIV(LFCLK_FREQ, MS_TIMER_FREQ)) - 1;

    NVIC_SetPriority(RTC_IRQN, irq_priority);
    NVIC_EnableIRQ(RTC_IRQN);
}

/**@todo Take care of values of ticks passed which are greater than 2^24, now it is
 *  suppressed to 2^24 when greater.
 * @warning Works only for input less than 512000 milli-seconds or 8.5 min when
 *  @ref MS_TIMER_FREQ is 32768. In other cases the max time must be calculated.
 */
void ms_timer_start(ms_timer_num id, ms_timer_mode mode, uint64_t ticks, void (*handler)(void))
{
    /* make sure the number of ticks to interrupt is less than 2^56 */
    ticks = ticks & 0x00FFFFFFFFFFFFFF;
    ASSERT((ticks == 0 && mode == MS_REPEATED_CALL) == false);
    if(ticks == 0)
    {
        ms_timer_stop(id);
        if(mode == MS_SINGLE_CALL)
        {
            handler();
            return;
        }
    }
    ticks = (ticks < 2) ? 2 : ticks;
    uint32_t counter_val = RTC_ID->COUNTER;

    ms_timer[id].timer_handler = handler;
    if (mode == MS_SINGLE_CALL)
    {
        ms_timer[id].timer_mode = MS_SINGLE_CALL;
    }
    else
    {
        ms_timer[id].timer_mode = ticks;
    }
    
    cal_overflow_ticks_req (counter_val, ticks, id);

    RTC_ID->EVENTS_COMPARE[id] = 0;
     
    if (ms_timers_status == 0)
    {
        RTC_ID->EVENTS_OVRFLW = 0;
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
#if ISR_MANAGER == 1
void ms_timer_rtc_Handler ()
#else
void RTC_IRQ_Handler()
#endif
{
    uint32_t counter_val = RTC_ID->COUNTER;
    if(RTC_ID->EVENTS_OVRFLW)
    {
#if ISR_MANAGER == 0
        RTC_ID->EVENTS_OVRFLW = 0;
#endif
        rtc_overflow_handler ();
    }
    for (ms_timer_num id = MS_TIMER0; id < MS_TIMER_MAX; id++)
    {
        if (RTC_ID->EVENTS_COMPARE[id])
        {
            RTC_ID->EVTENCLR = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
            RTC_ID->INTENCLR = 1 << (RTC_INTENSET_COMPARE0_Pos + id);
#if ISR_MANAGER == 0
            RTC_ID->EVENTS_COMPARE[id] = 0;
            (void)RTC_ID->EVENTS_COMPARE[id];
#endif

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
                cal_overflow_ticks_req (counter_val, ms_timer[id].timer_mode, id);
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
