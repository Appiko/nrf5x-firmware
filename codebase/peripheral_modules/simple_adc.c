/*
 *  simple_adc.c
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

#include "simple_adc.h"
#include "common_util.h"
#include "hal_pin_analog_input.h"

#define CHANNEL_USED    0

uint16_t saadc_result[1];

uint32_t simple_adc_get_value(simple_adc_gain_t gain, simple_adc_input_t pin)
{
    NRF_SAADC->TASKS_STOP = 1;
    saadc_result[0] = 0;
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
    NRF_SAADC->RESULT.PTR = (uint32_t) saadc_result;
    NRF_SAADC->RESULT.MAXCNT = 1;

    NRF_SAADC->CH[CHANNEL_USED].PSELP = pin;
    NRF_SAADC->CH[CHANNEL_USED].PSELN = SAADC_CH_PSELN_PSELN_NC;

    NRF_SAADC->CH[CHANNEL_USED].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos)
            & SAADC_CH_CONFIG_RESP_Msk)
            | ((SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) & SAADC_CH_CONFIG_RESN_Msk)
            | ((gain << SAADC_CH_CONFIG_GAIN_Pos) & SAADC_CH_CONFIG_GAIN_Msk)
            | ((SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos)
                    & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) & SAADC_CH_CONFIG_TACQ_Msk)
            | ((SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) & SAADC_CH_CONFIG_MODE_Msk)
            | ((SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos)
                    & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;

    NRF_SAADC->TASKS_START = 1;
    while(NRF_SAADC->EVENTS_STARTED == 0);

    NRF_SAADC->TASKS_SAMPLE = 1;
    while(NRF_SAADC->EVENTS_END == 0);

    NRF_SAADC->TASKS_STOP = 1;

    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_STARTED = 0;
    return saadc_result[0];
}

