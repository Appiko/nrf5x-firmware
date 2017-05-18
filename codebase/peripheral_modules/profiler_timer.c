/*
 *  profiler_timer.c
 *
 *  Created on: 15-May-2017
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

