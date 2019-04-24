/**
 *  us_timer.c : Microsecond timer
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

#include "us_timer.h"
#include "nrf_util.h"
#include <stddef.h>

/** @anchor timer_defines
 * @name Defines for the specific timer peripheral used for us timer
 * @{*/
#define TIMER_ID              CONCAT_2(NRF_TIMER, US_TIMER_USED)
#define TIMER_IRQN            CONCAT_3(TIMER, US_TIMER_USED, _IRQn)
#define TIMER_IRQ_Handler     CONCAT_3(TIMER, US_TIMER_USED, _IRQHandler)
#define TIMER_SIZE            CONCAT_3(NRF_TIMER, US_TIMER_USED, _MAX_SIZE)
/** @} */

/** Prescalar to the HF Clock for the TIMER peripheral based on f = HFCLK/(2^prescaler)
 *  With 4, the TIMER ticks every us. Also it'll use the PCLK1M lowering power used.*/
#define TIMER_PRESCALER    4
/** Number of bits in the timer. TIMER 1 and 2 in nRF51 can only be 8 or 16 bit */
#if(TIMER_SIZE ==32)
#define TIMER_BITSIZE      TIMER_BITMODE_BITMODE_32Bit
#else
#define TIMER_BITSIZE      TIMER_BITMODE_BITMODE_16Bit
#endif
/**
 * Structure to hold the @ref timer_mode and handler to be called upon expiry
 */
static struct us_timer_t{
    volatile uint32_t timer_mode;
    void (*timer_handler)(void);
}us_timer[US_TIMER_MAX];

/** Timers currently used based on the first four bits from LSB */
static volatile uint32_t us_timers_status;

void us_timer_init(uint32_t irq_priority){
    for(us_timer_num id = US_TIMER0; id < US_TIMER_MAX; id++){
        us_timer[id].timer_mode = US_SINGLE_CALL;
        us_timer[id].timer_handler  = NULL;
    }
    us_timers_status = 0;

    TIMER_ID->TASKS_STOP      = 1;                        // Stop timer.
    TIMER_ID->MODE            = TIMER_MODE_MODE_Timer;    // Set the timer in Timer Mode.
    TIMER_ID->PRESCALER       = TIMER_PRESCALER;         // Prescaler 4 produces 1 MHz.
    TIMER_ID->BITMODE         = TIMER_BITSIZE;           // 32 bit mode.
    TIMER_ID->TASKS_CLEAR     = 1;                        // clear the Timer first to be usable for later.
    TIMER_ID->TASKS_START     = 1;

    NVIC_SetPriority(TIMER_IRQN, irq_priority);
    NVIC_EnableIRQ(TIMER_IRQN);
}

void us_timer_start(us_timer_num id, us_timer_mode mode, uint32_t time_us, void (*handler)(void)){
    us_timer[id].timer_handler = handler;
    if(mode == US_SINGLE_CALL){
        us_timer[id].timer_mode  = US_SINGLE_CALL;
    }else{
        us_timer[id].timer_mode  = time_us;
    }

    TIMER_ID->TASKS_CAPTURE[id] = 1;
    TIMER_ID->CC[id] += time_us;

    TIMER_ID->EVENTS_COMPARE[id]  = 0;
    TIMER_ID->INTENSET            = 1 << (TIMER_INTENSET_COMPARE0_Pos + id);

    //If no timers are currently on
    if (us_timers_status == 0)
    {
        TIMER_ID->TASKS_START = 1;
    }

    us_timers_status |= 1 << id;
}

void us_timer_stop(us_timer_num id){
    us_timer[id].timer_mode  = US_SINGLE_CALL;
    us_timers_status &= ~(1 << id);

    TIMER_ID->INTENCLR        = 1 << (TIMER_INTENSET_COMPARE0_Pos + id);

    //If the stopped timer is the last timer that's on
    if (us_timers_status == 0)
    {
        TIMER_ID->TASKS_STOP          = 1;                // Stop timer.
        TIMER_ID->TASKS_SHUTDOWN      = 1;                // Fully stop timer.
    }
}

bool us_timer_get_on_status(us_timer_num id){
    return ((us_timers_status & (1<<id)) != 0);
}

/** @brief Function for handling the TIMER interrupts.
 * Triggered Compare register of timer ID
 */
void TIMER_IRQ_Handler(){
    for(us_timer_num id = US_TIMER0; id < US_TIMER_MAX; id++){
        if(TIMER_ID->EVENTS_COMPARE[id]){
            TIMER_ID->EVENTS_COMPARE[id] = 0;
            (void) TIMER_ID->EVENTS_COMPARE[id];

            void (*cb_handler)(void) = NULL;
            if(us_timer[id].timer_handler != NULL)
            {
                cb_handler = us_timer[id].timer_handler;
            }

            if (us_timer[id].timer_mode == US_SINGLE_CALL)
            {
                us_timer_stop(id);
            }
            else
            {
                TIMER_ID->TASKS_CAPTURE[id] = 1;
                TIMER_ID->CC[id] += us_timer[id].timer_mode;
            }

            if(cb_handler != NULL)
            {
                cb_handler();
            }
        }
    }
}

