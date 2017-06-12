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
#include "log.h"

void lfclk_init(lfclk_src_t lfclk_src)
{
    if((NRF_CLOCK->LFCLKSTAT &  //Clock is running
       (CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos)) &&
       //Correct source is already set
      ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_SRC_Msk) == lfclk_src))
    {
        //Already in the required configuration
        return;
    }

    NRF_CLOCK->LFCLKSRC = (lfclk_src << CLOCK_LFCLKSRC_SRC_Msk);
    NRF_CLOCK->INTENSET = CLOCK_INTENSET_LFCLKSTARTED_Msk;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

    // Enable wake-up on any event or interrupt (even disabled)
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    (void)NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    /* Wait for the external oscillator to start up. */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        __WFE();
    }
    /* Clear the event and the pending interrupt */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    (void)NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
    NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_LFCLKSTARTED_Msk;

    // Enable wake-up on only enabled interrupts
    SCB->SCR &= (~(SCB_SCR_SEVONPEND_Msk));
#ifdef NRF52832
        //Due to errata 20 in Eng rev 1
        NRF_RTC0->TASKS_STOP = 0;
        NRF_RTC1->TASKS_STOP = 0;
        NRF_RTC2->TASKS_STOP = 0;
#endif
}

void lfclk_deinit(void)
{
    NRF_CLOCK->TASKS_LFCLKSTOP = 1;
}

void hfclk_xtal_init_blocking(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if(NRF_CLOCK->HFCLKSTAT !=
      ((CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) |
      (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
    {
        NRF_CLOCK->INTENSET = CLOCK_INTENSET_HFCLKSTARTED_Msk;
        // Enable wake-up on event
        SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        (void) NRF_CLOCK->EVENTS_HFCLKSTARTED;
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        /* Wait for the external oscillator to start up. */
        while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        {
            __WFE();
        }
        /* Clear the event and the pending interrupt */
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        (void) NRF_CLOCK->EVENTS_HFCLKSTARTED;
        NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
        NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_HFCLKSTARTED_Msk;

        // Enable wake-up on only enabled interrupts
        SCB->SCR &= (~(SCB_SCR_SEVONPEND_Msk));
    }
}

void hfclk_xtal_init_nonblocking(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if(NRF_CLOCK->HFCLKSTAT !=
      ((CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) |
      (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
    }
}

void hfclk_block_till_xtal(void)
{
    /* Check if 16 MHz crystal oscillator is already running. */
    if(NRF_CLOCK->HFCLKSTAT !=
      ((CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) |
      (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
    {
        /* Wait for the external oscillator to start up. */
        while (NRF_CLOCK->HFCLKSTAT !=
            ((CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) |
            (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
        {

        }
        /* Clear the event and the pending interrupt */
        NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
        (void) NRF_CLOCK->EVENTS_HFCLKSTARTED;
    }
}

void hfclk_xtal_deinit(void)
{
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
}
