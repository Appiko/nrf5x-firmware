/**
 *  hal_wdt.c : Watchdog timer HAL
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

#include "nrf.h"
#include "stddef.h"
#include "hal_wdt.h"
#include "nrf_util.h"
#include "common_util.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

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

#if ISR_MANAGER == 1
void hal_wdt_Handler ()
#else
void WDT_IRQHandler(void)
#endif
{
#if ISR_MANAGER == 0
    NRF_WDT->EVENTS_TIMEOUT = 0;
#endif
    if (wdt_irq_handler != NULL)
    {
        wdt_irq_handler();
    }
}
