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
 * @brief Logging data from the SAADC peripheral
 * @{
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"

#include "boards.h"
#include "hal_saadc.h"
#include "hal_clocks.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "nrf_assert.h"
#include "ms_timer.h"
#include "nrf_util.h"
#include "nrf_delay.h"
#include "nrf_ppi.h"
#include "common_util.h"

/*      Defines         */
/** @brief Macro that defines the number of channels of ADC to sample */
#define SAADC_NUMBER_OF_CHANNELS    2

/** @anchor saadc_channel_configs
 * @name Macros for setting the SAADC channel configurations for this application.
 * @{*/
#define SAADC_WIDE_CHANNEL_CONFIG                      \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN4,                 \
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,    \
        .acq_time   = NRF_SAADC_ACQTIME_10US,          \
        .mode       = NRF_SAADC_MODE_DIFFERENTIAL,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = PIN_TO_ANALOG_INPUT(WIDE_SIGNAL_PIN),      \
        .pin_n      = PIN_TO_ANALOG_INPUT(WIDE_OFFSET_PIN)       \
    }

#define SAADC_NARROW_CHANNEL_CONFIG                    \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN1,                 \
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,    \
        .acq_time   = NRF_SAADC_ACQTIME_10US,          \
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = PIN_TO_ANALOG_INPUT(NARROW_SIGNAL_PIN),      \
        .pin_n      = PIN_TO_ANALOG_INPUT(NARROW_OFFSET_PIN)       \
    }
/** @} */

/** @anchor saadc_channel_limits
 * @name Macros for setting the SAADC channel limits for this application.
 * @{*/
#define NARROW_CHANNEL_UPPER_LIMIT    (800)
#define NARROW_CHANNEL_LOWER_LIMIT    (-800)

#define WIDE_CHANNEL_UPPER_LIMIT      (800)
#define WIDE_CHANNEL_LOWER_LIMIT      (-800)
/** @} */

/** @brief The ADC converstion resolution used in this application */
#define APPLN_SAADC_RESOLUTION    NRF_SAADC_RESOLUTION_12BIT
/** @brief The number of samples used to average to get the result */
#define APPLN_SAADC_OVERSAMPLING  NRF_SAADC_OVERSAMPLE_4X
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
        { NARROW_CHANNEL_UPPER_LIMIT, NARROW_CHANNEL_LOWER_LIMIT },
        { WIDE_CHANNEL_UPPER_LIMIT, WIDE_CHANNEL_LOWER_LIMIT }
};

/** @brief The array of SAADC channel configurations stored in flash used in initialization */
static const nrf_saadc_channel_config_t saadc_ch_config[SAADC_NUMBER_OF_CHANNELS] =
    { SAADC_WIDE_CHANNEL_CONFIG, SAADC_NARROW_CHANNEL_CONFIG };

/*      Globals        */
/** @brief The single length array that stores the SAADC converted value */
static int16_t saadc_result[1];

/*      Function declarations        */
/** @brief Sets up the SAADC peripheral according to its configuration defines,
 *      initializes the RTC to trigger the SAADC and starts the periodic sampling.
 */
static void saadc_init(void);

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
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
//        SEGGER_RTT_printf(0, "END: %d [%d%d] %d [%d%d]\n", saadc_result[0], NRF_SAADC->EVENTS_CH[0].LIMITH, NRF_SAADC->EVENTS_CH[0].LIMITL,
//                saadc_result[1], NRF_SAADC->EVENTS_CH[1].LIMITH, NRF_SAADC->EVENTS_CH[1].LIMITL );
        SEGGER_RTT_printf(0, "END: %d\n", saadc_result[0]);
        NRF_SAADC->EVENTS_CH[0].LIMITH = NRF_SAADC->EVENTS_CH[0].LIMITL = 0;
        NRF_SAADC->EVENTS_CH[1].LIMITH = NRF_SAADC->EVENTS_CH[1].LIMITL = 0;
    }
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
    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH0_LIMITH))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH0_LIMITH);
        SEGGER_RTT_printf(0, "CH0_LIMITH: %d %d\n", saadc_result[0], saadc_result[1]);
    }

    if(nrf_saadc_event_check(NRF_SAADC_EVENT_CH0_LIMITL))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_CH0_LIMITL);
        SEGGER_RTT_printf(0, "CH0_LIMITL: %d %d\n", saadc_result[0], saadc_result[1]);
    }
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

    nrf_saadc_int_enable(NRF_SAADC_INT_CH0LIMITH |
                         NRF_SAADC_INT_CH0LIMITL |
                         NRF_SAADC_INT_CH1LIMITH |
                         NRF_SAADC_INT_CH1LIMITL);

    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL0,
            nrf_saadc_event_address_get(NRF_SAADC_EVENT_STARTED),
            nrf_saadc_task_address_get(NRF_SAADC_TASK_SAMPLE));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL1,
            nrf_rtc_event_address_get(NRF_RTC2, NRF_RTC_EVENT_COMPARE_0),
            nrf_saadc_task_address_get(NRF_SAADC_TASK_START));
    nrf_ppi_fork_endpoint_setup(NRF_PPI_CHANNEL1,
            nrf_rtc_task_address_get(NRF_RTC2, NRF_RTC_TASK_CLEAR));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL1);

    nrf_ppi_channel_endpoint_setup(NRF_PPI_CHANNEL2,
            nrf_saadc_event_address_get(NRF_SAADC_EVENT_END),
            nrf_saadc_task_address_get(NRF_SAADC_TASK_STOP));
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL2);

    NRF_RTC2->TASKS_STOP = 1;
    NRF_RTC2->PRESCALER = 0;
    NRF_RTC2->CC[0] = RTC_TICKS_MS(20);
    NRF_RTC2->EVTENSET = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    NRF_RTC2->EVENTS_COMPARE[0] = 0;
    NRF_RTC2->TASKS_START = 1;
    nrf_saadc_enable();
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Mandatory welcome message */
    SEGGER_RTT_printf(0, "\n\nHello World over RTT!\n");

    hfclk_xtal_init_blocking();
//    lfclk_init(BOARD_LFCLKSRC);

    saadc_init();

    while (true)
    {
        __WFI();
    }
}

/** @} */
/** @} */
