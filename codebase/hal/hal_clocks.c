/**
 *  hal_clocks.c : Clocks HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hal_clocks.h"
#include "log.h"
#include "nrf_peripherals.h"

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

    NRF_CLOCK->LFCLKSRC = (lfclk_src << CLOCK_LFCLKSRC_SRC_Pos);
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

    //Due to errata
    NRF_RTC0->TASKS_STOP = 0;
    NRF_RTC1->TASKS_STOP = 0;
#if RTC_COUNT == 3
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

uint32_t hfclk_xtal_get_src ()
{
    return (NRF_CLOCK->HFCLKSTAT & 0x01);
}
