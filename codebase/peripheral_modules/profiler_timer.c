/**
 *  profiler_timer.c : Profiler timer
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

#include "profiler_timer.h"

void profiler_timer_init(void)
{
    PROFILER_TIMER->TASKS_STOP = 1;                         // Stop timer.
    PROFILER_TIMER->MODE = TIMER_MODE_MODE_Timer;           // Set the timer in Timer Mode.
    PROFILER_TIMER->PRESCALER = 0;                          // Prescaler 0 produces 16 MHz.
    PROFILER_TIMER->BITMODE = TIMER_BITMODE_BITMODE_32Bit;  // 32 bit mode.
    PROFILER_TIMER->TASKS_CLEAR = 1;                 // clear the task first to be usable for later.

    PROFILER_TIMER->TASKS_START = 1;                        // Start timer.
}

void profiler_timer_deinit()
{
    PROFILER_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;  // 32 bit mode.

    PROFILER_TIMER->TASKS_STOP = 1;                         // Stop timer.
    PROFILER_TIMER->TASKS_SHUTDOWN = 1;                     // Fully stop timer.
}

bool profiler_timer_is_on(void)
{
    return (PROFILER_TIMER->BITMODE == TIMER_BITMODE_BITMODE_16Bit)? false : true;
}

void printfcomma(uint32_t num)
{
    if (num < 1000)
    {
        log_printf("%d", (int) num);
        return;
    }
    printfcomma(num / 1000);
    log_printf(",%03d", (int) num % 1000);
}

