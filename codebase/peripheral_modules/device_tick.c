/*
 *  device_tick.c
 *
 *  Created on: 22-Mar-2018
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

#include "device_tick.h"
#include "irq_msg_util.h"
#include "ms_timer.h"
#include "nrf_assert.h"

#define LFCLK_TICKS_TO_DEV_TICKS(lfclk_ticks)  (lfclk_ticks/DEVICE_TICK_LFCLK_DIV_FACTOR)
#define DEV_TICKS_TO_LFCLK_TICKS(dev_ticks)    (dev_ticks*DEVICE_TICK_LFCLK_DIV_FACTOR)

struct {
  device_tick_mode current_mode;
  uint32_t half_current_interval;
  uint32_t fast_tick_interval;
  uint32_t slow_tick_interval;
  uint32_t last_tick_count;
}device_tick_ctx;

static void add_tick(void){
  uint32_t current_count, duration;

  current_count = ms_timer_get_current_count();
  duration = (current_count + (1<<24) - device_tick_ctx.last_tick_count) & 0xFFFFFF;
  device_tick_ctx.last_tick_count = current_count;
  irq_msg_push(NEXT_INTERVAL,(void *) (uint32_t) LFCLK_TICKS_TO_DEV_TICKS(duration));
}

void tick_timer_handler(void)
{
    add_tick();
}

void device_tick_init(device_tick_cfg * cfg)
{
    ASSERT(cfg->fast_mode_ticks < cfg->slow_mode_ticks);

    device_tick_ctx.current_mode = cfg->mode;
    device_tick_ctx.fast_tick_interval = cfg->fast_mode_ticks;
    device_tick_ctx.slow_tick_interval = cfg->slow_mode_ticks;

    device_tick_ctx.half_current_interval = ((cfg->mode == DEVICE_TICK_SLOW)?
            cfg->slow_mode_ticks:cfg->fast_mode_ticks)/2;

    ms_timer_start(MS_TIMER0, MS_REPEATED_CALL,
            2*device_tick_ctx.half_current_interval, tick_timer_handler);
    add_tick();
}

void device_tick_switch_mode(device_tick_mode mode)
{
    device_tick_ctx.half_current_interval = ((mode == DEVICE_TICK_SLOW)?
            device_tick_ctx.slow_tick_interval:
            device_tick_ctx.fast_tick_interval)/2;

    ms_timer_start(MS_TIMER0, MS_REPEATED_CALL,
            2*device_tick_ctx.half_current_interval, tick_timer_handler);
    add_tick();
}

void device_tick_process(void)
{
    uint32_t current_count, duration;

    current_count = ms_timer_get_current_count();
    duration = (current_count + (1<<24) - device_tick_ctx.last_tick_count) & 0xFFFFFF;

    if(duration > device_tick_ctx.half_current_interval)
    {
        ms_timer_start(MS_TIMER0, MS_REPEATED_CALL,
                2*device_tick_ctx.half_current_interval, tick_timer_handler);

        device_tick_ctx.last_tick_count = current_count;
        irq_msg_push(NEXT_INTERVAL,
                (void *) (uint32_t) LFCLK_TICKS_TO_DEV_TICKS(duration));
    }
}
