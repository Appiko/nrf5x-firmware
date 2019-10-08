/**
 *  pir_sense.c : PIR sensor driver
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

#include "pir_sense.h"
#include "nrf.h"
#include "common_util.h"
#include "nrf_util.h"
#include "hal_ppi.h"
#include "aux_clk.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

/** Specify which RTC peripheral would be used for the PIR Sense module */
#define PIR_SENSE_RTC_USED           RTC_USED_PIR_SENSE

#define SAADC_CHANNEL                SAADC_CHANNEL_USED_PIR_SENSE

#define RTC_ID                       CONCAT_2(NRF_RTC,RTC_USED_PIR_SENSE)

/** @brief The single length array that stores the SAADC converted value */
static int16_t saadc_result[1];

/** @brief The callback handler */
void (*sense_handler)(int32_t adc_val);

/** @brief Implementation of the SAADC interrupt handler */
#if ISR_MANAGER == 1
void pir_sense_saadc_Handler (void)
#else
void SAADC_IRQHandler(void)
#endif
{
#if ISR_MANAGER == 0
    NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITH = 0;
    (void) NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITH;
    NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITL = 0;
    (void) NRF_SAADC->EVENTS_CH[SAADC_CHANNEL].LIMITL;
#endif
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
                | ((SAADC_CH_CONFIG_TACQ_40us        << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
                | ((SAADC_CH_CONFIG_MODE_Diff        << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
                | ((SAADC_CH_CONFIG_BURST_Disabled   << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[SAADC_CHANNEL].LIMIT = (
                (((-1*((int16_t) init->threshold)) << SAADC_CH_LIMIT_LOW_Pos) & SAADC_CH_LIMIT_LOW_Msk)
              | (((uint16_t) init->threshold << SAADC_CH_LIMIT_HIGH_Pos) & SAADC_CH_LIMIT_HIGH_Msk));

    NVIC_SetPriority(SAADC_IRQn, init->irq_priority);
    NVIC_EnableIRQ(SAADC_IRQn);
    NVIC_ClearPendingIRQ(SAADC_IRQn);

    NRF_SAADC->INTENCLR = 0xFFFFFFFF;
    NRF_SAADC->INTENSET = ((1 << (SAADC_CHANNEL*2 + 6)) | (1 << (SAADC_CHANNEL*2 + 7)));

    //On ADC start event, trigger the ADC sampling
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_1].EEP = (uint32_t) &(NRF_SAADC->EVENTS_STARTED);
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_1].TEP = (uint32_t) &(NRF_SAADC->TASKS_SAMPLE);
    
    hal_ppi_setup_t pir_ppi1 = 
    {
        .ppi_id = PPI_CHANNEL_USED_PIR_SENSE_1,
        .event = (uint32_t) &(NRF_SAADC->EVENTS_STARTED),
        .task = (uint32_t) &(NRF_SAADC->TASKS_SAMPLE),
    };
    hal_ppi_set (&pir_ppi1);

    //On RTC0 Compare0 event, trigger ADC start task and also clear the RTC0 counter
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_2].EEP = (uint32_t) &(RTC_ID->EVENTS_COMPARE[0]);
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_2].TEP = (uint32_t) &(NRF_SAADC->TASKS_START);
//    NRF_PPI->FORK[PPI_CHANNEL_USED_PIR_SENSE_2].TEP = (uint32_t) &(RTC_ID->TASKS_CLEAR);
    

    //On ADC end event, stop the ADC to disable it and save power
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_3].EEP = (uint32_t) &(NRF_SAADC->EVENTS_END);
//    NRF_PPI->CH[PPI_CHANNEL_USED_PIR_SENSE_3].TEP = (uint32_t) &(NRF_SAADC->TASKS_STOP);

    hal_ppi_setup_t pir_ppi3 = 
    {
        .ppi_id = PPI_CHANNEL_USED_PIR_SENSE_3,
        .event = (uint32_t) &(NRF_SAADC->EVENTS_STOPPED),
        .task = (uint32_t) &(NRF_SAADC->TASKS_STOP),
    };
    hal_ppi_set (&pir_ppi3);

    //Enable the above three PPIs
//    NRF_PPI->CHENSET = (1 << PPI_CHANNEL_USED_PIR_SENSE_1) |
//            (1 << PPI_CHANNEL_USED_PIR_SENSE_2) |
//            (1 << PPI_CHANNEL_USED_PIR_SENSE_3);
//
//    //Start off the ADC with CC0 as the sensing interval
//    RTC_ID->TASKS_STOP = 1;
//    RTC_ID->PRESCALER = 0;
//    RTC_ID->CC[0] = LFCLK_TICKS_MS(init->sense_interval_ms);
//    RTC_ID->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
//    RTC_ID->EVENTS_COMPARE[0] = 0;
//    RTC_ID->TASKS_START = 1;

    hal_ppi_en_ch (PPI_CHANNEL_USED_PIR_SENSE_1);
    hal_ppi_en_ch (PPI_CHANNEL_USED_PIR_SENSE_3);
    aux_clk_setup_t aux_clk_setup = 
    {
        .arr_cc_ms[0] = (init->sense_interval_ms),
        .arr_ppi_cnf[0].event = AUX_CLK_EVT_CC0,
        .arr_ppi_cnf[0].task1 = (uint32_t) &(NRF_SAADC->TASKS_START),
        .arr_ppi_cnf[0].task2 = AUX_CLK_TASKS_CLEAR,
        .source = init->clk_src,
        .events_en = 0,
        .callback_handler = 0,
    };
    aux_clk_set (&aux_clk_setup);
    aux_clk_start ();
    
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
    NRF_SAADC->CH[SAADC_CHANNEL].PSELP = SAADC_CH_PSELP_PSELP_NC;

//    NRF_PPI->CHENCLR = (PPI_CHENCLR_CH0_Clear << PPI_CHANNEL_USED_PIR_SENSE_1) |
//                (PPI_CHENCLR_CH1_Clear << PPI_CHANNEL_USED_PIR_SENSE_2) |
//                (PPI_CHENCLR_CH2_Clear << PPI_CHANNEL_USED_PIR_SENSE_3);
//
//    RTC_ID->TASKS_CLEAR = 1;
//    RTC_ID->TASKS_STOP = 1;
    
    hal_ppi_dis_ch (PPI_CHANNEL_USED_PIR_SENSE_1);
    hal_ppi_dis_ch (PPI_CHANNEL_USED_PIR_SENSE_3);
    aux_clk_dis_ppi_ch (PPI_CHANNEL_USED_PIR_SENSE_2);
    aux_clk_stop ();
    aux_clk_clear ();
}

void pir_sense_update_threshold (uint32_t threshold)
{

    NRF_SAADC->CH[SAADC_CHANNEL].LIMIT = (
                (((-1*((int16_t) threshold)) << SAADC_CH_LIMIT_LOW_Pos) & SAADC_CH_LIMIT_LOW_Msk)
              | (((uint16_t) threshold << SAADC_CH_LIMIT_HIGH_Pos) & SAADC_CH_LIMIT_HIGH_Msk));
    
}

void pir_sense_switch_clock (pir_sense_clk_t clk_src)
{
    aux_clk_select_src ((aux_clk_source_t)clk_src);
}

