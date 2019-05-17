/**
 *  hal_pwm.c : PWM HAL
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

#include "hal_pwm.h"
#include "nrf_assert.h"
#include "hal_gpio.h"
#include "stddef.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

/** @anchor pwm_defines
 * @name Defines for the specific PWM peripheral used
 * @{*/
#define PWM_ID               CONCAT_2(NRF_PWM,PWM_USED)
#define PWM_IRQN             PWM_IRQN_a(PWM_USED)
#define PWM_IRQ_Handler      PWM_IRQ_Handler_a(PWM_USED)

#define PWM_IRQN_a(n)        PWM_IRQN_b(n)
#define PWM_IRQN_b(n)        PWM##n##_IRQn

#define PWM_IRQ_Handler_a(n) PWM_IRQ_Handler_b(n)
#define PWM_IRQ_Handler_b(n) PWM##n##_IRQHandler
/** @} */

static struct
{
    uint32_t pin_num;
    uint32_t irq_mask;
    void (*handler)(hal_pwm_irq_mask_t irq_source);
}cxt;

static void call_handler(hal_pwm_irq_mask_t irq_source)
{
    if(cxt.handler != NULL)
    {
        cxt.handler(irq_source);
    }
}
#if ISR_MANAGER == 1
void hal_pwm_Handler (void)
#else
void PWM_IRQ_Handler(void)
#endif
{
    if(PWM_ID->EVENTS_STOPPED == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_STOPPED = 0;
        (void) PWM_ID->EVENTS_STOPPED;
#endif
        call_handler(HAL_PWM_IRQ_STOPPED_MASK);
    }

    if(PWM_ID->EVENTS_SEQSTARTED[0] == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_SEQSTARTED[0] = 0;
        (void) PWM_ID->EVENTS_SEQSTARTED[0];
#endif
        call_handler(HAL_PWM_IRQ_SEQSTARTED0_MASK);
    }

    if(PWM_ID->EVENTS_SEQSTARTED[1] == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_SEQSTARTED[1] = 0;
        (void) PWM_ID->EVENTS_SEQSTARTED[1];
#endif
        call_handler(HAL_PWM_IRQ_SEQSTARTED1_MASK);
    }

    if(PWM_ID->EVENTS_SEQEND[0] == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_SEQEND[0] = 0;
        (void) PWM_ID->EVENTS_SEQEND[0];
#endif
        call_handler(HAL_PWM_IRQ_SEQEND0_MASK);
    }

    if(PWM_ID->EVENTS_SEQEND[1] == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_SEQEND[1] = 0;
        (void) PWM_ID->EVENTS_SEQEND[1];
#endif
        call_handler(HAL_PWM_IRQ_SEQEND1_MASK);
    }

    if(PWM_ID->EVENTS_PWMPERIODEND == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_PWMPERIODEND = 0;
        (void) PWM_ID->EVENTS_PWMPERIODEND;
#endif
        call_handler(HAL_PWM_IRQ_PWMPERIODEND_MASK);
    }

    if(PWM_ID->EVENTS_LOOPSDONE == 1)
    {
#if ISR_MANAGER == 0
        PWM_ID->EVENTS_LOOPSDONE = 0;
        (void) PWM_ID->EVENTS_LOOPSDONE;
#endif
        call_handler(HAL_PWM_IRQ_LOOPSDONE_MASK);
    }
}

void hal_pwm_init(hal_pwm_init_t * init_config)
{
    ASSERT((init_config->pin_num > 0)
            && (init_config->pin_num <= HAL_PWM_MAX_PIN_NUM));

    cxt.pin_num = init_config->pin_num;

    //Pins can only be assigned when PWM is disabled
    PWM_ID->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);

    PWM_ID->INTEN = 0;
    PWM_ID->TASKS_STOP = 1;

    switch(init_config->pin_num)
    {
    case 1:
        hal_gpio_cfg_high_output(init_config->pins[0], init_config->pin_idle_state[0]);
        PWM_ID->PSEL.OUT[0] = (init_config->pins[0] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        PWM_ID->PSEL.OUT[1] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        PWM_ID->PSEL.OUT[2] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        PWM_ID->PSEL.OUT[3] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        break;
    case 2: //2 pin would be used with grouped decoder loading
        hal_gpio_cfg_high_output(init_config->pins[0], init_config->pin_idle_state[0]);
        PWM_ID->PSEL.OUT[0] = (init_config->pins[0] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        hal_gpio_cfg_high_output(init_config->pins[1], init_config->pin_idle_state[1]);
        PWM_ID->PSEL.OUT[2] = (init_config->pins[1] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        PWM_ID->PSEL.OUT[1] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        PWM_ID->PSEL.OUT[3] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        break;
    case 3:
    case 4:
        //Configure the pins as output with the appropriate idle state
        for(uint32_t i = 0; i < init_config->pin_num; i++)
        {
            hal_gpio_cfg_high_output(init_config->pins[i], init_config->pin_idle_state[i]);
            PWM_ID->PSEL.OUT[i] = (init_config->pins[i] << PWM_PSEL_OUT_PIN_Pos) |
                    (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        }
        //The rest of the channels of PWM are unused
        for(uint32_t i = init_config->pin_num; i < HAL_PWM_MAX_PIN_NUM; i++)
        {
            PWM_ID->PSEL.OUT[i] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                    (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        }
        break;
    default:
        break;
    }

    PWM_ID->PRESCALER = (init_config->oper_freq <<
            PWM_PRESCALER_PRESCALER_Pos);
    PWM_ID->MODE = (init_config->oper_mode << PWM_MODE_UPDOWN_Pos);

    NVIC_SetPriority(PWM0_IRQn, init_config->irq_priority);
    NVIC_EnableIRQ(PWM0_IRQn);
}

void hal_pwm_start(hal_pwm_start_t * start_config)
{
    //Only 3 channels available in waveform decoder load mode
    ASSERT(((start_config->decoder_load == HAL_PWM_LOAD_WAVE_FORM)
            && (cxt.pin_num == HAL_PWM_MAX_PIN_NUM)) == false);

    PWM_ID->INTEN = 0;
    PWM_ID->TASKS_STOP = 1;

    PWM_ID->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    PWM_ID->COUNTERTOP = (start_config->countertop << PWM_COUNTERTOP_COUNTERTOP_Pos);
    PWM_ID->LOOP = (start_config->loop << PWM_LOOP_CNT_Pos);
    PWM_ID->DECODER = (start_config->decoder_load << PWM_DECODER_LOAD_Pos) |
            (start_config->decoder_trigger << PWM_DECODER_MODE_Pos);
    PWM_ID->SHORTS = start_config->shorts_mask;

    PWM_ID->SEQ[0].PTR = ((uint32_t) start_config->seq_config[0].seq_values
            << PWM_SEQ_PTR_PTR_Pos);
    PWM_ID->SEQ[0].CNT = (start_config->seq_config[0].len) << PWM_SEQ_CNT_CNT_Pos;
    PWM_ID->SEQ[0].REFRESH = start_config->seq_config[0].repeats - 1;
    PWM_ID->SEQ[0].ENDDELAY = start_config->seq_config[0].end_delay;

    PWM_ID->SEQ[1].PTR = ((uint32_t) start_config->seq_config[1].seq_values
            << PWM_SEQ_PTR_PTR_Pos);
    PWM_ID->SEQ[1].CNT = (start_config->seq_config[1].len) << PWM_SEQ_CNT_CNT_Pos;
    PWM_ID->SEQ[1].REFRESH = start_config->seq_config[1].repeats - 1;
    PWM_ID->SEQ[1].ENDDELAY = start_config->seq_config[1].end_delay;

    cxt.irq_mask = start_config->interrupt_masks;
    cxt.handler = start_config->irq_handler;

    //No interrupts when this is zero
    PWM_ID->INTEN = start_config->interrupt_masks;

    PWM_ID->TASKS_SEQSTART[0] = 1;
}

void hal_pwm_stop(void)
{
    PWM_ID->INTEN = 0;
    PWM_ID->TASKS_STOP = 1;
    PWM_ID->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
}
