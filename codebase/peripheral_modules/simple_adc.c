/**
 *  simple_adc.c : A simple ADC driver
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

#include "simple_adc.h"
#include "common_util.h"
#include "hal_pin_analog_input.h"

int16_t saadc_result[1];

uint32_t simple_adc_get_value(simple_adc_gain_t gain, simple_adc_input_t pin)
{
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    NVIC_DisableIRQ(SAADC_IRQn);

    //Disable all interrupts
    NRF_SAADC->INTENCLR = 0xFFFFFFFF;

    NRF_SAADC->TASKS_STOP = 1;
    saadc_result[0] = 0;
    NRF_SAADC->EVENTS_END = 0;
    NRF_SAADC->EVENTS_STARTED = 0;

    NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;
    NRF_SAADC->RESULT.PTR = (uint32_t) saadc_result;
    NRF_SAADC->RESULT.MAXCNT = 1;

    NRF_SAADC->CH[SIMPLE_ADC_CHANNEL_USED].PSELP = pin;
    NRF_SAADC->CH[SIMPLE_ADC_CHANNEL_USED].PSELN = SAADC_CH_PSELN_PSELN_NC;

    NRF_SAADC->CH[SIMPLE_ADC_CHANNEL_USED].CONFIG = ((SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos)
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

    NRF_SAADC->EVENTS_CH[SIMPLE_ADC_CHANNEL_USED].LIMITH = 0;
    NRF_SAADC->EVENTS_CH[SIMPLE_ADC_CHANNEL_USED].LIMITL = 0;

    //An issue with the nRF52's SAADC where it'll give negative values
    //when the signal to the ADC is close to 0V
    if(saadc_result[0] < 0)
    {
        saadc_result[0] = 0;
    }
    NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos);
    NRF_SAADC->CH[SIMPLE_ADC_CHANNEL_USED].PSELP = SAADC_CH_PSELP_PSELP_NC;

    return  (uint32_t) saadc_result[0];
}

