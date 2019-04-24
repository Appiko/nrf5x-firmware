/**
 *  device_tick.c : Device Tick generator
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

#include "device_tick.h"
#include "irq_msg_util.h"
#include "ms_timer.h"
#include "nrf_assert.h"
#include "common_util.h"

#define DEV_TICK_MSTIMER    CONCAT_2(MS_TIMER,MS_TIMER_USED_DEVICE_TICKS)

#define MSTIMER_TICKS_TO_DEV_TICKS(lfclk_ticks)  (lfclk_ticks/DEVICE_TICK_MSTIMER_DIV_FACTOR)
#define DEV_TICKS_TO_MSTIMER_TICKS(dev_ticks)    (dev_ticks*DEVICE_TICK_MSTIMER_DIV_FACTOR)

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
  irq_msg_push(MSG_NEXT_INTERVAL,(void *) (uint32_t) MSTIMER_TICKS_TO_DEV_TICKS(duration));
}

void tick_timer_handler(void)
{
    add_tick();
}

void device_tick_init(device_tick_cfg * cfg)
{
    ASSERT(cfg->fast_mode_ticks < cfg->slow_mode_ticks);

    if(cfg->mode != DEVICE_TICK_SAME)
    {
        device_tick_ctx.current_mode = cfg->mode;
    }
    device_tick_ctx.fast_tick_interval = cfg->fast_mode_ticks;
    device_tick_ctx.slow_tick_interval = cfg->slow_mode_ticks;

    device_tick_ctx.half_current_interval =
            ((device_tick_ctx.current_mode == DEVICE_TICK_SLOW)?
            cfg->slow_mode_ticks:cfg->fast_mode_ticks)/2;

    ms_timer_start(DEV_TICK_MSTIMER, MS_REPEATED_CALL,
            2*device_tick_ctx.half_current_interval,
            tick_timer_handler);
    add_tick();
}

void device_tick_switch_mode(device_tick_mode mode)
{
    ASSERT(mode != DEVICE_TICK_SAME);

    if(device_tick_ctx.current_mode != mode){
        device_tick_ctx.current_mode = mode;
        device_tick_ctx.half_current_interval = ((mode == DEVICE_TICK_SLOW)?
                device_tick_ctx.slow_tick_interval:
                device_tick_ctx.fast_tick_interval)/2;

        ms_timer_start(DEV_TICK_MSTIMER, MS_REPEATED_CALL,
                2*device_tick_ctx.half_current_interval, tick_timer_handler);
        add_tick();
    }
}

void device_tick_process(void)
{
    uint32_t current_count, duration;

    current_count = ms_timer_get_current_count();
    duration = (current_count + (1<<24) - device_tick_ctx.last_tick_count) & 0xFFFFFF;

    if(duration > device_tick_ctx.half_current_interval)
    {
        ms_timer_start(DEV_TICK_MSTIMER, MS_REPEATED_CALL,
                2*device_tick_ctx.half_current_interval, tick_timer_handler);

        device_tick_ctx.last_tick_count = current_count;
        irq_msg_push(MSG_NEXT_INTERVAL,
                (void *) (uint32_t) MSTIMER_TICKS_TO_DEV_TICKS(duration));
    }
}
