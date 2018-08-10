/*
 *  pir_sense.c
 *
 *  Created on: 13-Feb-2018
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

#include "pir_sense.h"
#include "nrf.h"
#include "common_util.h"
#include "nrf_util.h"

/** Specify which RTC peripheral would be used for the PIR Sense module */
#define PIR_SENSE_RTC_USED           0

#define SAADC_CHANNEL                0

/** @brief The single length array that stores the SAADC converted value */
static int16_t saadc_result[1];

/** @brief The callback handler */
void (*sense_handler)(int32_t adc_val);

/** @brief Implementation of the SAADC interrupt handler */
void SAADC_IRQHandler(void)
{
    NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITH = 0;
    (void) NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITH;
    NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITL = 0;
    (void) NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITL;

    sense_handler(saadc_result[0]);
}

void pir_sense_start(pir_sense_cfg * init)
{
    //Set the handler to be called
    sense_handler = init->handler;

    //ADC config: 12 bit, no oversampling, differential inputs, 10 us sampling,
    //no burst mode, 1.2V internal reference
    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
    NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    NRF_SAADC->SAMPLERATE = (0 << SAADC_SAMPLERATE_CC_Pos) |
                (SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos);
    NRF_SAADC->RESULT.PTR = (uint32_t)saadc_result;
    NRF_SAADC->RESULT.MAXCNT = ARRAY_SIZE(saadc_result);
    NRF_SAADC->CH[SAADC_CHANNEL].PSELN = init->pir_offset_analog_in;
    NRF_SAADC->CH[SAADC_CHANNEL].PSELP = init->pir_signal_analog_in;

    NRF_SAADC->CH[SAADC_CHANNEL].CONFIG =
                ((SAADC_CH_CONFIG_RESP_Bypass        << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
                | ((SAADC_CH_CONFIG_RESN_Bypass      << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
                | ((SAADC_CH_CONFIG_GAIN_Gain1       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
                | ((SAADC_CH_CONFIG_REFSEL_Internal  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
                | ((SAADC_CH_CONFIG_TACQ_10us        << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                | ((SAADC_CH_CONFIG_MODE_Diff        << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
                | ((SAADC_CH_CONFIG_BURST_Disabled   << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[SAADC_CHANNEL].LIMIT = (
                (((-1*((int16_t) init->threshold)) << SAADC_CH_LIMIT_LOW_Pos) & SAADC_CH_LIMIT_LOW_Msk)
              | (((uint16_t) init->threshold << SAADC_CH_LIMIT_HIGH_Pos) & SAADC_CH_LIMIT_HIGH_Msk));

    NVIC_SetPriority(SAADC_IRQn, init->irq_priority);
    NVIC_EnableIRQ(SAADC_IRQn);
    NVIC_ClearPendingIRQ(SAADC_IRQn);

    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NRF_SAADC->INTENSET = (SAADC_INTENSET_CH0LIMITH_Msk | SAADC_INTENSET_CH0LIMITL_Msk );

    //On ADC start event, trigger the ADC sampling
    NRF_PPI->CH[PPI_CHEN_CH0_Pos].EEP = (uint32_t) &(NRF_SAADC->EVENTS_STARTED);
    NRF_PPI->CH[PPI_CHEN_CH0_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_SAMPLE);

    //On RTC0 Compare0 event, trigger ADC start task and also clear the RTC0 counter
    NRF_PPI->CH[PPI_CHEN_CH1_Pos].EEP = (uint32_t) &(NRF_RTC0->EVENTS_COMPARE[0]);
    NRF_PPI->CH[PPI_CHEN_CH1_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_START);
    NRF_PPI->FORK[PPI_CHEN_CH1_Pos].TEP = (uint32_t) &(NRF_RTC0->TASKS_CLEAR);

    //On ADC end event, stop the ADC to disable it and save power
    NRF_PPI->CH[PPI_CHEN_CH2_Pos].EEP = (uint32_t) &(NRF_SAADC->EVENTS_END);
    NRF_PPI->CH[PPI_CHEN_CH2_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_STOP);

    //Enable the above three PPIs
    NRF_PPI->CHENSET = (PPI_CHENSET_CH0_Set << PPI_CHEN_CH0_Pos) |
            (PPI_CHENSET_CH1_Set << PPI_CHEN_CH1_Pos) |
            (PPI_CHENSET_CH2_Set << PPI_CHEN_CH2_Pos);

    //Start off the ADC with CC0 as the sensing interval
    NRF_RTC0->TASKS_STOP = 1;
    NRF_RTC0->PRESCALER = 0;
    NRF_RTC0->CC[0] = LFCLK_TICKS_MS(init->sense_interval_ms);
    NRF_RTC0->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    NRF_RTC0->TASKS_START = 1;

    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos);
}

/**
 * @brief Disable the peripherals involved - SAADC, PPIs and RTC0
 */
void pir_sense_stop(void)
{
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
    NVIC_DisableIRQ(SAADC_IRQn);

    NRF_SAADC->INTENCLR = 0xFFFFFFFF;

    NRF_PPI->CHENCLR = (PPI_CHENCLR_CH0_Clear << PPI_CHEN_CH0_Pos) |
                (PPI_CHENCLR_CH1_Clear << PPI_CHEN_CH1_Pos) |
                (PPI_CHENCLR_CH2_Clear << PPI_CHEN_CH2_Pos);

    NRF_RTC0->TASKS_CLEAR = 1;
    NRF_RTC0->TASKS_STOP = 1;
}
