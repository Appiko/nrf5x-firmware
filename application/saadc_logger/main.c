/*  Copyright (c) 2016, Appiko
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

/**
 * @addtogroup group_appln
 * @{
 *
 * @defgroup saadc_logger SAADC Logger
 * @brief Low power analog data logger with the SAADC peripheral.
 *  The image below gives the flow of how the peripherals of nRF52 is setup for
 *  this application.
 *
 *
 * @dot
 * digraph State_machine_diagram {
 *  rankdir="LR";
 *  rtc_trg [shape = point]
 *  adc_trg [shape = point]
 *  rtc_evt [shape = circle, width = 1.5, label = "RTC\ntimeout"]
 *  rtc_clr [shape = circle, width = 1.5, label ="Clear RTC"]
 *  adc_start [shape = circle, width = 1.5, label ="Start ADC"]
 *  adc_sample [shape = circle, width = 1.5, label ="Sample ADC"]
 *  adc_end [shape = circle, width = 1.5, label = "ADC Ended"]
 *  adc_isr [shape = circle, width = 1.5, label = "ADC IRQ"]
 *  adc_stop [shape = circle, width = 1.5, label ="Stop ADC"]
 *  rtc_trg -> rtc_evt [style = "dotted", label = "Event on RTC\ncompare every\nsampling interval"]
 *  rtc_evt -> rtc_clr [label = "PPI:start counting\nagain from 0"];
 *  rtc_evt -> adc_start [label = "PPI fork:\nready the ADC"];
 *  adc_start -> adc_sample [label = "PPI:sample\nwith ADC"];
 *  adc_sample -> adc_end [style = "dotted", label="Event on\nsampling"];
 *  adc_sample -> adc_isr [style = "invisible", dir = "none"];
 *  adc_trg -> adc_isr [style = "dotted", label = "ISR:when ADC high/low\nlimits are crossed"]
 *  adc_end -> adc_stop [label = "PPI:stop ADC\non sampling"];
 * }
 * @enddot
 *
 * @{
 */
#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

#include "boards.h"
#include "hal_saadc.h"
#include "hal_clocks.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "nrf_assert.h"
#include "hal_nop_delay.h"
#include "ms_timer.h"
#include "common_util.h"
#include "nrf_util.h"
#include "log.h"

/*      Defines         */
/** @brief Macro that defines the data sampling frequency */
#define LOGGER_INTERVAL_MS          50

/** @brief Macro that defines the number of channels of ADC to sample */
#define SAADC_NUMBER_OF_CHANNELS    1

/** @anchor saadc_channel_configs
 * @name Macros for setting the SAADC channel configurations for this application.
 * @{*/
#if 0
#define SAADC_RC_CHANNEL_CONFIG                      \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN4,                 \
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,    \
        .acq_time   = NRF_SAADC_ACQTIME_10US,          \
        .mode       = NRF_SAADC_MODE_DIFFERENTIAL,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = PIN_TO_ANALOG_INPUT(PIR_RC_SIGNAL_PIN),      \
        .pin_n      = PIN_TO_ANALOG_INPUT(PIR_RC_OFFSET_PIN)       \
    }
#else
#define SAADC_AMP_CHANNEL_CONFIG                    \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN1,                 \
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,    \
        .acq_time   = NRF_SAADC_ACQTIME_10US,          \
        .mode       = NRF_SAADC_MODE_DIFFERENTIAL,     \
        .burst      = NRF_SAADC_BURST_DISABLED,         \
        .pin_p      = PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),      \
        .pin_n      = PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN)       \
    }
/** @} */
#endif

/** @anchor saadc_channel_limits
 * @name Macros for setting the SAADC channel limits for this application.
 * @{*/
#define PIR_RC_CHANNEL_UPPER_LIMIT      (32767)
#define PIR_RC_CHANNEL_LOWER_LIMIT      (-32768)

#define PIR_AMP_CHANNEL_UPPER_LIMIT    (100)
#define PIR_AMP_CHANNEL_LOWER_LIMIT    (-100)
/** @} */

/** @brief The ADC converstion resolution used in this application */
#define APPLN_SAADC_RESOLUTION    NRF_SAADC_RESOLUTION_12BIT
/** @brief The number of samples used to average to get the result */
#define APPLN_SAADC_OVERSAMPLING  NRF_SAADC_OVERSAMPLE_DISABLED
/** @brief The interrupt priority of the SAADC peripheral */
#define APPLN_SAADC_IRQ_PRIORITY  APP_IRQ_PRIORITY_LOW

/** @brief Configuration structure of the SAADC limit for a channel */
struct saadc_limit
{
    int16_t upper_limit;
    int16_t lower_limit;
};


/*      Global constants in flash         */
/** @brief The array of SAADC channel limits stored in flash used in initialization */
static const struct saadc_limit saadc_limit_config[SAADC_NUMBER_OF_CHANNELS] =
{
//        { PIR_RC_CHANNEL_UPPER_LIMIT, PIR_RC_CHANNEL_LOWER_LIMIT }
        { PIR_AMP_CHANNEL_UPPER_LIMIT, PIR_AMP_CHANNEL_LOWER_LIMIT }
};

/** @brief The array of SAADC channel configurations stored in flash used in initialization */
static const nrf_saadc_channel_config_t saadc_ch_config[SAADC_NUMBER_OF_CHANNELS] =
    {
//            SAADC_RC_CHANNEL_CONFIG
            SAADC_AMP_CHANNEL_CONFIG
    };

/*      Globals        */
/** @brief The single length array that stores the SAADC converted value */
static int16_t saadc_result[1];

/*      Function declarations        */
/** @brief Sets up the SAADC peripheral according to its configuration defines,
 *      initializes the RTC to trigger the SAADC and starts the periodic sampling.
 */
static void saadc_init(void);


void trigger_handler(void)
{
    log_printf("RTC handlers\n");
    hal_gpio_pin_toggle(LED_RED);
}


/*      Function definitions        */
/** @brief Implementation of the SAADC interrupt handler */
void SAADC_IRQHandler(void)
{
#if 0
    if(nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
        SEGGER_RTT_printf(0, "STARTED: %d\n", saadc_result);
    }

#endif
#if 1
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);

        log_printf("END: %d [%d%d]\n", saadc_result[0], NRF_SAADC->EVENTS_CH[0].LIMITH, NRF_SAADC->EVENTS_CH[0].LIMITL
//                ,saadc_result[1], NRF_SAADC->EVENTS_CH[1].LIMITH, NRF_SAADC->EVENTS_CH[1].LIMITL
                );
//        if(NRF_SAADC->EVENTS_CH[0].LIMITH){
//            SEGGER_RTT_printf(0, "END 0: %d\n", saadc_result[0]);
//        } else if (NRF_SAADC->EVENTS_CH[1].LIMITH){
//            SEGGER_RTT_printf(0, "END 1: %d\n", saadc_result[0]);
//        }

        NRF_SAADC->EVENTS_CH[0].LIMITH = NRF_SAADC->EVENTS_CH[0].LIMITL = 0;
        NRF_SAADC->EVENTS_CH[1].LIMITH = NRF_SAADC->EVENTS_CH[1].LIMITL = 0;
    }
#endif
#if 0

    if(nrf_saadc_event_check(NRF_SAADC_EVENT_DONE))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_DONE);
        SEGGER_RTT_printf(0, "DONE: %d\n", saadc_result);
    }

    if(nrf_saadc_event_check(NRF_SAADC_EVENT_RESULTDONE))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
        SEGGER_RTT_printf(0, "RESULTDONE: %d\n", saadc_result);
    }

    if(nrf_saadc_event_check(NRF_SAADC_EVENT_STOPPED))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
        SEGGER_RTT_printf(0, "STOPPED: %d %d\n", saadc_result[0], saadc_result[1]);
        nrf_saadc_disable();
    }
#endif
#if 1
    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH0_LIMITH))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH0_LIMITH);
        log_printf("CH0_LIMITH: %d\n", saadc_result[0]);
//        SEGGER_RTT_printf(0, "CH0_LIMITH: %d\n", saadc_result[0]);
//        ms_timer_start(MS_TIMER3,MS_SINGLE_CALL, RTC_TICKS_MS(49), trigger_handler);
//        hal_gpio_pin_write(LED_RED, LEDS_ACTIVE_STATE);
    }
    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH0_LIMITL))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH0_LIMITL);
        log_printf("CH0_LIMITL: %d\n", saadc_result[0]);
//        SEGGER_RTT_printf(0, "CH0_LIMITL: %d\n", saadc_result[0]);
//        ms_timer_start(MS_TIMER3,MS_SINGLE_CALL, RTC_TICKS_MS(49), trigger_handler);
//        hal_gpio_pin_write(LED_RED, LEDS_ACTIVE_STATE);
    }
#endif
#if 0
    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH1_LIMITH))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH1_LIMITH);
        SEGGER_RTT_printf(0, "CH1_LIMITH: %d %d\n", saadc_result[0], saadc_result[1]);
    }

    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH1_LIMITL))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH1_LIMITL);
        SEGGER_RTT_printf(0, "CH1_LIMITL: %d %d\n", saadc_result[0], saadc_result[1]);
    }
#endif
}


static void saadc_init(void)
{
    static_assert((SAADC_NUMBER_OF_CHANNELS > 0) && (SAADC_NUMBER_OF_CHANNELS <= 8),
            "The number of SAADC channels is between 1 and 8");
    static_assert((PIR_AMP_CHANNEL_LOWER_LIMIT >= -32768) && (PIR_AMP_CHANNEL_LOWER_LIMIT <= 32767),
            "The channel limit of SAADC is between -32768 and 32767");
    static_assert((PIR_AMP_CHANNEL_UPPER_LIMIT >= -32768) && (PIR_AMP_CHANNEL_UPPER_LIMIT <= 32767),
            "The channel limit of SAADC is between -32768 and 32767");
    static_assert((PIR_RC_CHANNEL_LOWER_LIMIT >= -32768) && (PIR_RC_CHANNEL_LOWER_LIMIT <= 32767),
            "The channel limit of SAADC is between -32768 and 32767");
    static_assert((PIR_RC_CHANNEL_UPPER_LIMIT >= -32768) && (PIR_RC_CHANNEL_UPPER_LIMIT <= 32767),
            "The channel limit of SAADC is between -32768 and 32767");

    nrf_saadc_resolution_set(APPLN_SAADC_RESOLUTION);
    saadc_sampling_task_mode_set();
    nrf_saadc_buffer_init(saadc_result, ARRAY_SIZE(saadc_result));
    NVIC_SetPriority(SAADC_IRQn, APPLN_SAADC_IRQ_PRIORITY);
    NVIC_EnableIRQ(SAADC_IRQn);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    nrf_saadc_oversample_set(APPLN_SAADC_OVERSAMPLING);

    for (uint32_t i = 0; i < SAADC_NUMBER_OF_CHANNELS; i++)
    {
        saadc_channel_init(i, &saadc_ch_config[i]);
        nrf_saadc_channel_limits_set(i,
                saadc_limit_config[i].lower_limit,
                saadc_limit_config[i].upper_limit);
    }

    nrf_saadc_int_enable(
                         NRF_SAADC_INT_END |
                         NRF_SAADC_INT_CH0LIMITH |
                         NRF_SAADC_INT_CH0LIMITL
//                         NRF_SAADC_INT_CH1LIMITH |
//                         NRF_SAADC_INT_CH1LIMITL
                         );

    NRF_PPI->CH[PPI_CHEN_CH0_Pos].EEP = (uint32_t) &(NRF_SAADC->EVENTS_STARTED);
    NRF_PPI->CH[PPI_CHEN_CH0_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_SAMPLE);
    NRF_PPI->CHENSET = PPI_CHENSET_CH0_Set << PPI_CHEN_CH0_Pos;

//    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL0,
//            nrf_saadc_event_address_get(NRF_SAADC_EVENT_STARTED),
//            nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
//    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

    NRF_PPI->CH[PPI_CHEN_CH1_Pos].EEP = (uint32_t) &(NRF_RTC0->EVENTS_COMPARE[0]);
    NRF_PPI->CH[PPI_CHEN_CH1_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_START);
    NRF_PPI->FORK[PPI_CHEN_CH1_Pos].TEP = (uint32_t) &(NRF_RTC0->TASKS_CLEAR);
    NRF_PPI->CHENSET = PPI_CHENSET_CH1_Set << PPI_CHEN_CH1_Pos;

//    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL1,
//            nrf_rtc_event_address_get(NRF_RTC0, NRF_RTC_EVENT_COMPARE_0),
//            nrf_saadc_task_address_get(NRF_SAADC_TASK_START));
//    nrf_ppi_fork_endpoint_setup(NRF_PPI_CHANNEL1,
//            nrf_rtc_task_address_get(NRF_RTC0, NRF_RTC_TASK_CLEAR));
//    nrf_ppi_channel_enable(NRF_PPI_CHANNEL1);

    NRF_PPI->CH[PPI_CHEN_CH2_Pos].EEP = (uint32_t) &(NRF_SAADC->EVENTS_END);
    NRF_PPI->CH[PPI_CHEN_CH2_Pos].TEP = (uint32_t) &(NRF_SAADC->TASKS_STOP);
    NRF_PPI->CHENSET = PPI_CHENSET_CH2_Set << PPI_CHEN_CH2_Pos;

//    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL2,
//            nrf_saadc_event_address_get(NRF_SAADC_EVENT_END),
//            nrf_saadc_task_address_get(NRF_SAADC_TASK_STOP));
//    nrf_ppi_channel_enable(NRF_PPI_CHANNEL2);

    NRF_RTC0->TASKS_STOP = 1;
    NRF_RTC0->PRESCALER = 0;
    NRF_RTC0->CC[0] = RTC_TICKS_MS(LOGGER_INTERVAL_MS);
    NRF_RTC0->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    NRF_RTC0->TASKS_START = 1;
    nrf_saadc_enable();
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Mandatory welcome message */
    log_init();
    log_printf("\n\nHello World!\n");

    lfclk_init(LFCLK_SRC_Xtal);

    saadc_init();

    while (true)
    {
        __WFI();
    }
}

/** @} */
/** @} */
