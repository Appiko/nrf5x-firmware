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

void PWM0_IRQHandler(void)
{
    if(NRF_PWM0->EVENTS_STOPPED == 1)
    {
        NRF_PWM0->EVENTS_STOPPED = 0;
        (void) NRF_PWM0->EVENTS_STOPPED;
        call_handler(HAL_PWM_IRQ_STOPPED_MASK);
    }

    if(NRF_PWM0->EVENTS_SEQSTARTED[0] == 1)
    {
        NRF_PWM0->EVENTS_SEQSTARTED[0] = 0;
        (void) NRF_PWM0->EVENTS_SEQSTARTED[0];
        call_handler(HAL_PWM_IRQ_SEQSTARTED0_MASK);
    }

    if(NRF_PWM0->EVENTS_SEQSTARTED[1] == 1)
    {
        NRF_PWM0->EVENTS_SEQSTARTED[1] = 0;
        (void) NRF_PWM0->EVENTS_SEQSTARTED[1];
        call_handler(HAL_PWM_IRQ_SEQSTARTED1_MASK);
    }

    if(NRF_PWM0->EVENTS_SEQEND[0] == 1)
    {
        NRF_PWM0->EVENTS_SEQEND[0] = 0;
        (void) NRF_PWM0->EVENTS_SEQEND[0];
        call_handler(HAL_PWM_IRQ_SEQEND0_MASK);
    }

    if(NRF_PWM0->EVENTS_SEQEND[1] == 1)
    {
        NRF_PWM0->EVENTS_SEQEND[1] = 0;
        (void) NRF_PWM0->EVENTS_SEQEND[1];
        call_handler(HAL_PWM_IRQ_SEQEND1_MASK);
    }

    if(NRF_PWM0->EVENTS_PWMPERIODEND == 1)
    {
        NRF_PWM0->EVENTS_PWMPERIODEND = 0;
        (void) NRF_PWM0->EVENTS_PWMPERIODEND;
        call_handler(HAL_PWM_IRQ_PWMPERIODEND_MASK);
    }

    if(NRF_PWM0->EVENTS_LOOPSDONE == 1)
    {
        NRF_PWM0->EVENTS_LOOPSDONE = 0;
        (void) NRF_PWM0->EVENTS_LOOPSDONE;
        call_handler(HAL_PWM_IRQ_LOOPSDONE_MASK);
    }
}

void hal_pwm_init(hal_pwm_init_t * init_config)
{
    ASSERT((init_config->pin_num > 0)
            && (init_config->pin_num <= HAL_PWM_MAX_PIN_NUM));

    cxt.pin_num = init_config->pin_num;

    //Pins can only be assigned when PWM is disabled
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);

    NRF_PWM0->INTEN = 0;
    NRF_PWM0->TASKS_STOP = 1;

    switch(init_config->pin_num)
    {
    case 1:
        hal_gpio_cfg_high_output(init_config->pins[0], init_config->pin_idle_state[0]);
        NRF_PWM0->PSEL.OUT[0] = (init_config->pins[0] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_PWM0->PSEL.OUT[1] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_PWM0->PSEL.OUT[2] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_PWM0->PSEL.OUT[3] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        break;
    case 2: //2 pin would be used with grouped decoder loading
        hal_gpio_cfg_high_output(init_config->pins[0], init_config->pin_idle_state[0]);
        NRF_PWM0->PSEL.OUT[0] = (init_config->pins[0] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        hal_gpio_cfg_high_output(init_config->pins[1], init_config->pin_idle_state[1]);
        NRF_PWM0->PSEL.OUT[2] = (init_config->pins[1] << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_PWM0->PSEL.OUT[1] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        NRF_PWM0->PSEL.OUT[3] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        break;
    case 3:
    case 4:
        //Configure the pins as output with the appropriate idle state
        for(uint32_t i = 0; i < init_config->pin_num; i++)
        {
            hal_gpio_cfg_high_output(init_config->pins[i], init_config->pin_idle_state[i]);
            NRF_PWM0->PSEL.OUT[i] = (init_config->pins[i] << PWM_PSEL_OUT_PIN_Pos) |
                    (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
        }
        //The rest of the channels of PWM are unused
        for(uint32_t i = init_config->pin_num; i < HAL_PWM_MAX_PIN_NUM; i++)
        {
            NRF_PWM0->PSEL.OUT[i] = (0 << PWM_PSEL_OUT_PIN_Pos) |
                    (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos);
        }
        break;
    default:
        break;
    }

    NRF_PWM0->PRESCALER = (init_config->oper_freq <<
            PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM0->MODE = (init_config->oper_mode << PWM_MODE_UPDOWN_Pos);

    NVIC_SetPriority(PWM0_IRQn, init_config->irq_priority);
    NVIC_EnableIRQ(PWM0_IRQn);
}

void hal_pwm_start(hal_pwm_start_t * start_config)
{
    //Only 3 channels available in waveform decoder load mode
    ASSERT(((start_config->decoder_load == HAL_PWM_LOAD_WAVE_FORM)
            && (cxt.pin_num == HAL_PWM_MAX_PIN_NUM)) == false);

    NRF_PWM0->INTEN = 0;
    NRF_PWM0->TASKS_STOP = 1;

    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
    NRF_PWM0->COUNTERTOP = (start_config->countertop << PWM_COUNTERTOP_COUNTERTOP_Pos);
    NRF_PWM0->LOOP = (start_config->loop << PWM_LOOP_CNT_Pos);
    NRF_PWM0->DECODER = (start_config->decoder_load << PWM_DECODER_LOAD_Pos) |
            (start_config->decoder_trigger << PWM_DECODER_MODE_Pos);
    NRF_PWM0->SHORTS = start_config->shorts_mask;

    NRF_PWM0->SEQ[0].PTR = ((uint32_t) start_config->seq_config[0].seq_values
            << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[0].CNT = (start_config->seq_config[0].len) << PWM_SEQ_CNT_CNT_Pos;
    NRF_PWM0->SEQ[0].REFRESH = start_config->seq_config[0].repeats - 1;
    NRF_PWM0->SEQ[0].ENDDELAY = start_config->seq_config[0].end_delay;

    NRF_PWM0->SEQ[1].PTR = ((uint32_t) start_config->seq_config[1].seq_values
            << PWM_SEQ_PTR_PTR_Pos);
    NRF_PWM0->SEQ[1].CNT = (start_config->seq_config[1].len) << PWM_SEQ_CNT_CNT_Pos;
    NRF_PWM0->SEQ[1].REFRESH = start_config->seq_config[1].repeats - 1;
    NRF_PWM0->SEQ[1].ENDDELAY = start_config->seq_config[1].end_delay;

    cxt.irq_mask = start_config->interrupt_masks;
    cxt.handler = start_config->irq_handler;

    //No interrupts when this is zero
    NRF_PWM0->INTEN = start_config->interrupt_masks;

    NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

void hal_pwm_stop(void)
{
    NRF_PWM0->INTEN = 0;
    NRF_PWM0->TASKS_STOP = 1;
    NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Disabled << PWM_ENABLE_ENABLE_Pos);
}
