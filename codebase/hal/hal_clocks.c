/*
 *  clocks.c
 *
 *  Created on: 06-Feb-2017
 *
 *  Copyright (c) 2017, Appiko
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

#include "hal_clocks.h"
#include "nrf.h"

void lfclk_init(nrf_clock_lfclk_t lfclk_src)
{
    if (nrf_clock_lf_is_running() == false)
    {
        nrf_clock_lf_src_set(lfclk_src);
        nrf_clock_int_enable(NRF_CLOCK_INT_LF_STARTED_MASK);
        NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

        // Enable wake-up on any event or interrupt (even disabled)
        SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

        nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);
        /* Wait for the external oscillator to start up. */
        while (nrf_clock_lf_is_running() == false)
        {
            __WFE();
        }
        /* Clear the event and the pending interrupt */
        nrf_clock_event_clear(NRF_CLOCK_EVENT_LFCLKSTARTED);
        NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
        nrf_clock_int_disable(NRF_CLOCK_INT_LF_STARTED_MASK);

        // Enable wake-up on only enabled interrupts
        SCB->SCR &= (~(SCB_SCR_SEVONPEND_Msk));

#ifdef NRF52832
        //Due to errata 20 in Eng rev 1
        NRF_RTC0->TASKS_STOP = 0;
        NRF_RTC1->TASKS_STOP = 0;
        NRF_RTC2->TASKS_STOP = 0;
#endif
    }
}

void lfclk_deinit(void)
{
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}

void hfclk_xtal_init_blocking(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) == false)
    {
        nrf_clock_int_enable(NRF_CLOCK_INT_HF_STARTED_MASK);
        // Enable wake-up on event
        SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

        nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);
        /* Wait for the external oscillator to start up. */
        while (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) == false)
        {
            __WFE();
        }
        /* Clear the event and the pending interrupt */
        nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
        NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
        nrf_clock_int_disable(NRF_CLOCK_INT_HF_STARTED_MASK);

        // Enable wake-up on only enabled interrupts
        SCB->SCR &= (~(SCB_SCR_SEVONPEND_Msk));
    }
}

void hfclk_xtal_init_nonblocking(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) == false)
    {
        nrf_clock_task_trigger(NRF_CLOCK_TASK_HFCLKSTART);
    }
}

void hfclk_block_till_xtal(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) == false)
    {
        nrf_clock_int_enable(NRF_CLOCK_INT_HF_STARTED_MASK);
        // Enable wake-up on event
        SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

        nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
        /* Wait for the external oscillator to start up. */
        while (nrf_clock_hf_is_running(NRF_CLOCK_HFCLK_HIGH_ACCURACY) == false)
        {
            __WFE();
        }
        /* Clear the event and the pending interrupt */
        nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
        NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
        nrf_clock_int_disable(NRF_CLOCK_INT_HF_STARTED_MASK);

        // Enable wake-up on only enabled interrupts
        SCB->SCR &= (~(SCB_SCR_SEVONPEND_Msk));
    }
}

void hfclk_xtal_deinit(void)
{
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}
