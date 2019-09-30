/*
 *  aux_clk.c : Module to handle auxiliary clock used by different modules
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

#include "aux_clk.h"
#include "log.h"
#include "nrf.h"
#include "string.h"
#include "gpio_level_handler.h"
#include "hal_ppi.h"
#include "hal_clocks.h"
#include "common_util.h"

#define RTC_USED  CONCAT_2(NRF_RTC, AUX_CLK_LFCLK_RTC_USED) 


//#if RTC_USED != NRF_RTC0
//#error "RTC Used not set"
//#endif

#define RTC_IRQ_Handler_a(n) RTC_IRQ_Handler_b(n)
#define RTC_IRQ_Handler_b(n) RTC##n##_IRQHandler

#define RTC_IRQN_a(n)  RTC_IRQN_b(n)
#define RTC_IRQN_b(n)  RTC##n##_IRQn

#define RTC_IRQN RTC_IRQN_a(AUX_CLK_LFCLK_RTC_USED)
#define RTC_IRQ_Handler RTC_IRQ_Handler_a(AUX_CLK_LFCLK_RTC_USED)

#define RTC_TICKS_MS(x) (uint32_t)(ROUNDED_DIV((32768 * x),1000))


#define TIMER_USED CONCAT_2(NRF_TIMER, AUX_CLK_HFCLK_TIMER_USED)


#define TIMER_IRQ_Handler TIMER_IRQ_Handler_a(AUX_CLK_HFCLK_TIMER_USED)
#define TIMER_IRQ_Handler_a(n) TIMER_IRQ_Handler_b(n)
#define TIMER_IRQ_Handler_b(n) TIMER##n##_IRQHandler

#define TIMER_IRQN TIMER_IRQN_a(AUX_CLK_HFCLK_TIMER_USED)
#define TIMER_IRQN_a(n)  TIMER_IRQN_b(n)
#define TIMER_IRQN_b(n)  TIMER##n##_IRQn

#define TIMER_TICKS_MS(x) (x * 1000)


/** Function pointer buffer to store callback handler */
void (*callbac_buffer) (uint8_t events);

/** Global variable to store current clock source */
static aux_clk_source_t g_source = AUX_CLK_SRC_LFCLK;

static app_irq_priority_t g_irq_priority;

static aux_clk_ppi_t g_arr_ppi_cnf[AUX_CLK_PPI_CHANNELS_USED];

#if ISR_MANAGER == 1
void aux_clk_rtc_handler (void)
#else
void RTC_IRQ_Handler(void)
#endif
{
    uint8_t events = 0;
    if(RTC_USED->EVENTS_COMPARE[0] == 1)
    {
        events |= AUX_CLK_EVT_CC0;
#if isr_manager == 0
        RTC_USED->EVENTS_COMPARE[0] = 0;
#endif
    }
    if(RTC_USED->EVENTS_COMPARE[1] == 1)
    {
        events |= AUX_CLK_EVT_CC1;
#if isr_manager == 0
        RTC_USED->EVENTS_COMPARE[1] = 0;
#endif
    }
    if(RTC_USED->EVENTS_COMPARE[2] == 1)
    {
        events |= AUX_CLK_EVT_CC2;
#if isr_manager == 0
        RTC_USED->EVENTS_COMPARE[2] = 0;
#endif
    }
    if(RTC_USED->EVENTS_COMPARE[3] == 1)
    {
        events |= AUX_CLK_EVT_CC3;
#if isr_manager == 0
        RTC_USED->EVENTS_COMPARE[3] = 0;
#endif
    }
    callbac_buffer (events);
}

#if ISR_MANAGER == 1
void aux_clk_timer_handler (void)
#else
void TIMER_IRQ_Handler (void)
#endif
{
    uint8_t events = 0;
    if(TIMER_USED->EVENTS_COMPARE[0] == 1)
    {
        events |= AUX_CLK_EVT_CC0;
#if ISR_MANAGER == 0
        TIMER_USED->EVENTS_COMPARE[0] = 0;
#endif
    }
    if(TIMER_USED->EVENTS_COMPARE[1] == 1)
    {
        events |= AUX_CLK_EVT_CC1;
#if ISR_MANAGER == 0
        TIMER_USED->EVENTS_COMPARE[1] = 0;
#endif
    }
    if(TIMER_USED->EVENTS_COMPARE[2] == 1)
    {
        events |= AUX_CLK_EVT_CC2;
#if ISR_MANAGER == 0
        TIMER_USED->EVENTS_COMPARE[2] = 0;
#endif
    }
    if(TIMER_USED->EVENTS_COMPARE[3] == 1)
    {
        events |= AUX_CLK_EVT_CC3;
#if ISR_MANAGER == 0
        TIMER_USED->EVENTS_COMPARE[3] = 0;
#endif
    }
    callbac_buffer (events);
}

void set_timer ()
{
#if AUX_CLK_HFCLK_SOLO_MODULE == 1
    hfclk_xtal_init_blocking ();
#endif
    if(g_irq_priority != APP_IRQ_PRIORITY_THREAD)
    {
        NVIC_SetPriority (TIMER_IRQN, g_irq_priority);
        NVIC_EnableIRQ (TIMER_IRQN);
    }
    TIMER_USED->TASKS_START = 1;
    (void) TIMER_USED->TASKS_START;
}

void set_rtc ()
{
#if AUX_CLK_HFCLK_SOLO_MODULE == 1
    hfclk_xtal_deinit ();
#endif
    if(g_irq_priority != APP_IRQ_PRIORITY_THREAD)
    {
        NVIC_SetPriority (RTC_IRQN, g_irq_priority);
        NVIC_EnableIRQ (RTC_IRQN);
    }
    RTC_USED->TASKS_START = 1;
    (void) RTC_USED->TASKS_START;
}

uint32_t select_event (uint32_t evt)
{
    
    uint32_t event;
    switch (evt)
    {
        case AUX_CLK_EVT_NON : 
            event = 0;
            break;

        case AUX_CLK_EVT_CC0 : 
            event = (uint32_t)((g_source == AUX_CLK_SRC_LFCLK) ? 
                &RTC_USED->EVENTS_COMPARE[0] :
                &TIMER_USED->EVENTS_COMPARE[0]);
            break;

        case AUX_CLK_EVT_CC1 : 
            event = (uint32_t)((g_source == AUX_CLK_SRC_LFCLK) ? 
                &RTC_USED->EVENTS_COMPARE[1] :
                &TIMER_USED->EVENTS_COMPARE[1]);
            break;

        case AUX_CLK_EVT_CC2 : 
            event = (uint32_t)((g_source == AUX_CLK_SRC_LFCLK) ? 
                &RTC_USED->EVENTS_COMPARE[2] :
                &TIMER_USED->EVENTS_COMPARE[2]);
            break;

        case AUX_CLK_EVT_CC3 : 
            event = (uint32_t)((g_source == AUX_CLK_SRC_LFCLK) ? 
                &RTC_USED->EVENTS_COMPARE[3] :
                &TIMER_USED->EVENTS_COMPARE[3]);
            break;

        default :
            event = evt;
            break;
    }
    return event;
}

uint32_t select_task (uint32_t tsk)
{
    uint32_t task;
    switch (tsk)
    {
        case AUX_CLK_TASKS_START : 
            task = (uint32_t) ((g_source == AUX_CLK_SRC_LFCLK) ?
                &RTC_USED->TASKS_START :
                &TIMER_USED->TASKS_START);
            break;

        case AUX_CLK_TASKS_STOP : 
            task = (uint32_t) ((g_source == AUX_CLK_SRC_LFCLK) ?
                &RTC_USED->TASKS_STOP :
                &TIMER_USED->TASKS_STOP);
            break;

        case AUX_CLK_TASKS_CLEAR : 
            task =  ((g_source == AUX_CLK_SRC_LFCLK) ?
                (uint32_t) &RTC_USED->TASKS_CLEAR :
                (uint32_t) &TIMER_USED->TASKS_CLEAR);
            break;

        default :
            task = tsk; 
            break;
    }
    return task;
}

void set_ppi ()
{
    hal_ppi_setup_t ppi_setup;
    uint8_t ppi_status;
    for(uint32_t cnt = 0; cnt < AUX_CLK_PPI_CHANNELS_USED; cnt++)
    {
        ppi_setup.ppi_id = AUX_CLK_PPI_CHANNEL_BASE + cnt;
        ppi_setup.event = select_event (g_arr_ppi_cnf[cnt].event);
        ppi_setup.task = select_task (g_arr_ppi_cnf[cnt].task1);
        ppi_setup.fork = select_task (g_arr_ppi_cnf[cnt].task2);
        ppi_status = hal_ppi_set (&ppi_setup);
        if(ppi_status == PPI_SETUP_SUCCESSFUL)
        {
            hal_ppi_en_ch (ppi_setup.ppi_id);
        }
    }
}

void aux_clk_select_src (aux_clk_source_t source)
{
    g_source = source;
    NVIC_DisableIRQ (RTC_IRQN);
    NVIC_DisableIRQ (TIMER_IRQN); 
    RTC_USED->TASKS_CLEAR = 1;
    TIMER_USED->TASKS_CLEAR =1;
    for(uint32_t cnt = 0; cnt < AUX_CLK_MAX_CHANNELS; cnt++)
    {
        RTC_USED->EVENTS_COMPARE[cnt] = 0;
        TIMER_USED->EVENTS_COMPARE[cnt] = 0;
    }
    set_ppi ();
    (g_source == AUX_CLK_SRC_LFCLK) ? set_rtc () : set_timer ();

}

void aux_clk_set (aux_clk_setup_t * aux_clk)
{
    g_source = aux_clk->source;
    memcpy (g_arr_ppi_cnf, aux_clk->arr_ppi_cnf, 
            (sizeof(aux_clk_ppi_t) * AUX_CLK_PPI_CHANNELS_USED));
    callbac_buffer = aux_clk->callback_handler;
    
    RTC_USED->PRESCALER = 0;
    
    TIMER_USED->PRESCALER = 4;
    TIMER_USED->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    TIMER_USED->MODE = TIMER_MODE_MODE_Timer;
    
    for(uint32_t cnt = 0; cnt < AUX_CLK_MAX_CHANNELS; cnt++)
    {
        RTC_USED->CC[cnt] = RTC_TICKS_MS(aux_clk->arr_cc_ms[cnt]);
        TIMER_USED->CC[cnt] = TIMER_TICKS_MS(aux_clk->arr_cc_ms[cnt]);
        if((aux_clk->events_en & (uint8_t)(1<<cnt)))
        {
            RTC_USED->EVTENSET |= 1 << (16 + cnt);
            RTC_USED->INTENSET |= 1 << (16 + cnt);
            TIMER_USED->INTENSET |= 1 << (16 + cnt);
        }
       
    }
}

void aux_clk_start ()
{
    set_ppi ();
    (g_source == AUX_CLK_SRC_LFCLK) ? set_rtc () : set_timer ();
}

void aux_clk_stop ()
{
    NVIC_DisableIRQ (RTC_IRQN);
    RTC_USED->TASKS_CLEAR = 1;
    RTC_USED->TASKS_STOP = 1;
    NVIC_DisableIRQ (TIMER_IRQN);
    TIMER_USED->TASKS_CLEAR = 1;
    TIMER_USED->TASKS_STOP = 1;
    TIMER_USED->TASKS_SHUTDOWN = 1;    
    (void) TIMER_USED->TASKS_SHUTDOWN;
}
 
void aux_clk_clear ()
{
    TIMER_USED->TASKS_CLEAR = 1;
    RTC_USED->TASKS_CLEAR = 1;
}

void aux_clk_en_evt (uint8_t events)
{
    RTC_USED->EVTENSET |= (events << 16);
    RTC_USED->INTENSET |= (events << 16);        
    TIMER_USED->INTENSET |= (events << 16);
}

void aux_clk_dis_evt (uint8_t events)
{
    RTC_USED->EVTENCLR |= (events << 16);
    RTC_USED->INTENCLR |= (events << 16);        
    TIMER_USED->INTENCLR |= (events << 16);
}

void aux_clk_update_ppi (uint32_t ppi_channel, aux_clk_ppi_t * new_ppi)
{
    memcpy (&g_arr_ppi_cnf[ppi_channel - AUX_CLK_PPI_CHANNEL_BASE],
            new_ppi, sizeof(aux_clk_ppi_t));
    hal_ppi_setup_t ppi_update = 
    {
        .ppi_id = ppi_channel,
        .event = select_event (new_ppi->event),
        .task = select_task (new_ppi->task1),
        .fork = select_task (new_ppi->task2),
    }; 
    hal_ppi_set (&ppi_update);
}

void aux_clk_update_cc (uint32_t cc_id,uint32_t new_val_ms)
{
    TIMER_USED->CC[cc_id] = TIMER_TICKS_MS(new_val_ms);
    RTC_USED->CC[cc_id] = RTC_TICKS_MS(new_val_ms);
}

void aux_clk_update_irq_priority (app_irq_priority_t new_priority)
{
    g_irq_priority = new_priority;
}

uint32_t aux_clk_get_ms (void)
{
    uint32_t ms;
    
    if(g_source ==AUX_CLK_SRC_LFCLK)
    {
        ms = (RTC_USED->COUNTER * 1000)/32768;
    }
    else
    {
        uint32_t cc0 = TIMER_USED->CC[0];
        TIMER_USED->TASKS_CAPTURE[0] = 1;
        uint32_t counter = TIMER_USED->CC[0];
        TIMER_USED->CC[0] = cc0;
        ms = counter / 1000;
    }
    
    return ms;
}
