/*
 *  hal_wdt.c
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

#include "nrf.h"
#include "stddef.h"
#include "hal_wdt.h"
#include "nrf_util.h"
#include "common_util.h"

/** Special value to reset WDT, shouldn't be modified.*/
#define WDT_RR_VALUE       0x6E524635UL

void (*wdt_irq_handler)(void);

void hal_wdt_init(uint32_t period_ms, void (*wdt_timeout_handler)(void))
{
    //In case the WDT is already running
    if (NRF_WDT->RUNSTATUS == WDT_RUNSTATUS_RUNSTATUS_Running)
    {
        hal_wdt_feed();
    }
    else
    {
        //Run when sleeping, pause when debugging
        NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos)
                | (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

        NRF_WDT->CRV = LFCLK_TICKS_MS(period_ms);

        //Enable only RR1 and RR7. Two registers so it won't be randomly fed.
        NRF_WDT->RREN = (WDT_RREN_RR1_Enabled << WDT_RREN_RR1_Pos)
                | (WDT_RREN_RR7_Enabled << WDT_RREN_RR7_Pos);

        wdt_irq_handler = wdt_timeout_handler;

        NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Enabled;

        //APP_IRQ_PRIORITY_HIGH define works with or without SD in all nRF5x
        NVIC_ClearPendingIRQ(WDT_IRQn);
        NVIC_SetPriority(WDT_IRQn, APP_IRQ_PRIORITY_HIGH);
        NVIC_EnableIRQ(WDT_IRQn);
    }
}

void hal_wdt_feed(void)
{
    //Reload the two registers enabled.
    NRF_WDT->RR[1] = WDT_RR_VALUE;
    NRF_WDT->RR[7] = WDT_RR_VALUE;
}

void hal_wdt_start(void)
{
    NRF_WDT->TASKS_START = 1;
}

void WDT_IRQHandler(void)
{
    NRF_WDT->EVENTS_TIMEOUT = 0;

    if (wdt_irq_handler != NULL)
    {
        wdt_irq_handler();
    }
}
