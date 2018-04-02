/*
 *  main.c
 *
 *  Created on: 30-Jan-2018
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

/**
 * @addtogroup group_appln
 * @{
 *
 * @defgroup sense_appln The code for the PIR based Sense units.
 * @brief The PIR sense application's main file that makes it operate.
 *
 * @{
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "hal_clocks.h"

#include "log.h"
#include "hal_wdt.h"
#include "pir_sense.h"
#include "nrf_util.h"
#include "hal_pin_analog_input.h"
#include "hal_gpio.h"
#include "ms_timer.h"
#include "irq_msg_util.h"
#include "device_tick.h"

#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"

/*      Defines         */
/** The WDT bites if not fed every 600 sec (10 min) */
#define WDT_PERIOD_MS              600000

#define PIR_SENSE_INTERVAL_MS      50
#define PIR_SENSE_THRESHOLD        600

#define FAST_TICK_INTERVAL_MS      500
#define SLOW_TICK_INTERVAL_MS      5000

#define ENABLE_WDT                 1

typedef enum
{
    SENSING,
    ADVERTISING,
    CONNECTED
}sense_states;

/*      Global constants in flash         */
sense_states state;

/*      Function declarations        */
void pir_handler(int32_t adc_val);

/*      Function definitions        */

/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}

void jack_clear_handler(void)
{
    hal_gpio_pin_set(JACK_TRIGGER_PIN);
    hal_gpio_pin_set(JACK_FOCUS_PIN);
}

void pir_reset_handler(void)
{
    pir_sense_cfg cfg = {
            PIR_SENSE_INTERVAL_MS,
            PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
            PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
            PIR_SENSE_THRESHOLD,
            APP_IRQ_PRIORITY_HIGH,
            pir_handler
    };
    pir_sense_start(&cfg);
}

void pir_handler(int32_t adc_val)
{
    log_printf("Sensed %d\n", adc_val);

    ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, LFCLK_TICKS_MS(50),
            jack_clear_handler);
    ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, LFCLK_TICKS_MS(1000),
            pir_reset_handler);
    pir_sense_stop();
    hal_gpio_pin_clear(JACK_TRIGGER_PIN);
    hal_gpio_pin_clear(JACK_FOCUS_PIN);
}

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
static void ble_stack_init(void)
{
    uint32_t err_code;
    const nrf_clock_lf_cfg_t cfg = BOARD_LFCLKSRC_STRUCT;

    err_code = sd_softdevice_enable(&cfg, app_error_fault_handler);
    APP_ERROR_CHECK(err_code);
}

void next_interval_handler(uint32_t interval)
{
    log_printf("interval %d\n", interval);
}

void state_change_handler(uint32_t new_state)
{

}

void jack_pins_init(void)
{
    //Standard low output, float on high
    hal_gpio_cfg(JACK_TRIGGER_PIN,
            GPIO_PIN_CNF_DIR_Output,
            GPIO_PIN_CNF_INPUT_Disconnect,
            GPIO_PIN_CNF_PULL_Disabled,
            GPIO_PIN_CNF_DRIVE_S0D1,
            GPIO_PIN_CNF_SENSE_Disabled);
    hal_gpio_pin_set(JACK_TRIGGER_PIN);

    //Standard low output, float on high
    hal_gpio_cfg(JACK_FOCUS_PIN,
            GPIO_PIN_CNF_DIR_Output,
            GPIO_PIN_CNF_INPUT_Disconnect,
            GPIO_PIN_CNF_PULL_Disabled,
            GPIO_PIN_CNF_DRIVE_S0D1,
            GPIO_PIN_CNF_SENSE_Disabled);
    hal_gpio_pin_set(JACK_FOCUS_PIN);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    /* Mandatory welcome message */
    log_init();
    log_printf("\n\nHello Sense World! %d \n");

    lfclk_init(LFCLK_SRC_Xtal);
#if ENABLE_WDT == 1
    hal_wdt_init(WDT_PERIOD_MS, wdt_prior_reset_callback);
    hal_wdt_start();
#endif
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
    jack_pins_init();

    ble_stack_init();

    {
        irq_msg_callbacks cb =
            { next_interval_handler, state_change_handler };
        irq_msg_init(&cb);

        pir_sense_cfg cfg = {
                PIR_SENSE_INTERVAL_MS,
                PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
                PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
                PIR_SENSE_THRESHOLD,
                APP_IRQ_PRIORITY_HIGH,
                pir_handler
        };
        pir_sense_start(&cfg);
    }

    while (true)
    {
#if ENABLE_WDT == 1
        //Since the application demands that CPU wakes up
        hal_wdt_feed();
#endif
        device_tick_process();
        irq_msg_process();
        sd_app_evt_wait();
    }
}

/** @} */
/** @} */
