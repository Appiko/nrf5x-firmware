/*
 *  hw_testing_app.c : Tests for SensePi Board level testing
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

#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "nrf_util.h"

#include "hal_pin_analog_input.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "hal_clocks.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"

#include "profiler_timer.h"
#include "uart_printf.h"
#include "tinyprintf.h"
#include "log.h"

#include "hw_testing_app.h"
#include "KXTJ3.h"
#include "string.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "ms_timer.h"
#include "hal_spi_rf.h"


#define TIMER_INIT_VALUE (read_time_us())
#define CALC_TIMER_VAL 1000000
#define MAX_FREQ 10

/// ADC value for Vdd/2
#define CALC_OFFSET simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(BATT_VOLTAGE_SENSE))/2

/**
 * @brief This function is used to check if one number is in some percent window
 * of other number
 * @param data The variable which is to be checked in window
 * @param ref Window has to be around this number
 * @param per Percentage value for which window is to be built
 */
static uint32_t compare_percent(uint32_t data, uint32_t ref, float per);

/** The possible sources for Low frequency clock's 32 kHz input
 */
typedef enum {
    MY_LFCLK_SRC_RC = CLOCK_LFCLKSRC_SRC_RC, ///< Internal RC oscillator.
    MY_LFCLK_SRC_Xtal = CLOCK_LFCLKSRC_SRC_Xtal, ///< External crystal.
    MY_LFCLK_SRC_Synth = CLOCK_LFCLKSRC_SRC_Synth ///< Synthesizer from HFCLK clock.
} my_lfclk_src_t;

/** @brief Function to initialize the LF clock
 * @param lfclk_src The source for the lf clock (RC, Xtal or systhesis from HF-clk)
 * @warning If the clock source is RC oscillator, calibrate it to use (errata 77 in nRF52)
 */
static uint32_t my_lfclk_init(my_lfclk_src_t my_lfclk_src);

/**
 * @brief This function is an interrupt handler for MS_TIMER0
 */
void capture_lfclk_test(void);


static uint32_t timer_val_us;



//Supporting mathematical funcs

uint32_t compare_percent(uint32_t data, uint32_t ref, float per) {
    if ((ref - (ref * per / 100)) <= data && data <= (ref + (ref * per / 100))) {
        return 1;
    } else {
        return 0;
    }
}



//Funtions for actual tests.
// Power Test

uint32_t power_test(void) {
    log_printf("TEST 1:Power 1\n");
    hal_nop_delay_ms(100);
    return 1;
}
// DC/DC Test 

uint32_t dc_dc_test(void) {
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;
    log_printf("TEST 2: DC_DC %d\n", NRF_POWER->DCDCEN);
    return (NRF_POWER->DCDCEN);
}

//Crystal test 

/**
 * @note 1 tick for MS_TIMER0 is 31 us long. And all the initialization takes 247 us.
 */

static uint32_t my_lfclk_init(my_lfclk_src_t my_lfclk_src) {
    uint32_t my_xtal_flag = 0;
    if ((NRF_CLOCK->LFCLKSTAT & //Clock is running
            (CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos)) &&
            //Correct source is already set
            ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_SRC_Msk) == my_lfclk_src)) {
        //Already in the required configuration
        my_xtal_flag = 1;
        return my_xtal_flag;
    }

    NRF_CLOCK->LFCLKSRC = (my_lfclk_src << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->INTENSET = CLOCK_INTENSET_LFCLKSTARTED_Msk;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

    // Enable wake-up on any event or interrupt (even disabled)
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    (void) NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    hal_nop_delay_ms(500);
    /* Wait for the external oscillator to start up. */
    if (NRF_CLOCK->EVENTS_LFCLKSTARTED == 1) {
        my_xtal_flag = 1;
    }
    /* Clear the event and the pending interrupt */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    (void) NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
    NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_LFCLKSTARTED_Msk;

    return my_xtal_flag;

}

uint32_t crystal_test() {
    uint32_t flag = 0;
    uint32_t check_status_xtal = 0;
    ms_timer_init(1);
    check_status_xtal = my_lfclk_init(MY_LFCLK_SRC_Xtal);
    ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, 32768, capture_lfclk_test);
    profiler_timer_init();
    hal_nop_delay_ms(1020);
    flag = compare_percent(timer_val_us, CALC_TIMER_VAL, 1);
    ms_timer_stop(1);
    check_status_xtal
            ? log_printf("Test 3.1:Found Crystal 1\n")
            : log_printf("Test 3.1:Found Crystal 0\n");

    if (flag && check_status_xtal) {
        log_printf("Test 3.2:Crystal accurate 1\n");
        return 1;
    } else {
        log_printf("Test 3.2:Crystal accurate 0\n");
        return 0;
    }

}

void capture_lfclk_test(void) {
    timer_val_us = read_time_us();
    log_printf("INFO 3.2 : 32768 ticks observed in %d us\n", timer_val_us);
    profiler_timer_deinit();
}

uint32_t accelerometer_test(void) {
    KXTJ3_g_data_t g_acce_data;

    KXTJ3_config_t kxtj_init = {
        .callback_handler = NULL,
        .gpio_intr = ACCE_INT_PIN,
        .i2c_sck = SCK_PIN,
        .i2c_sda = SDA_PIN,
        .range = KXTJ_RNG_2g,
        .resolution = KXTJ_RES_8Bit,
    };
    kxtj3_init(&kxtj_init);
    kxtj3_start();
    hal_nop_delay_ms(100);
    memcpy(&g_acce_data, kxtj3_get_acce_value(), sizeof (KXTJ3_g_data_t));
    if ((g_acce_data.zg > 900 && g_acce_data.zg < 1100) || (g_acce_data.zg < -900 && g_acce_data.zg > -1100)) {
        log_printf("TEST 4: Accelerometer 1\n");
        return 1;
    }
    log_printf("TEST 4: Accelerometer 0\n");
    return 0;
}

uint32_t cc1175_clk_test(void) {
    
    if (get_device_id() != 10) { //unknown device
        log_printf("TEST 5: CLOCK TEST 0\n");
        return 0;
    }
    hal_gpio_cfg_output(TCXO_EN_PIN, 0);
    hal_gpio_pin_set(TCXO_EN_PIN);
    radio_init(APPIKO_1120_0K3);
    
    if (get_device_id() ==  20) { //unknown device
        log_printf("TEST 5: CLOCK TEST 1\n");
        return 1;
    }
    
    log_printf("TEST 5: CLOCK END 0\n");
    return 0;
}

uint32_t cc1175_transmission_test(void) {
    hal_nop_delay_ms(1000);
    uint8_t data[] = {1};

    radio_set_freq(915000);

    set_rf_packet_length(1);
    radio_send(data, 1);
    log_printf("TEST 6: Transmitted 1\n");
    return 1;
}

uint32_t hall_effect_test(void) {
    hal_gpio_cfg_input(HALL_EFFECT_PIN, HAL_GPIO_PULL_DOWN);
    
    profiler_timer_init();
    int states[] = {1,0,1};
    int checkCount = 0;
    while(read_time_us() < 3000000 && checkCount < 3) 
    {
        if(hal_gpio_pin_read(HALL_EFFECT_PIN) == states[checkCount])
        {
            checkCount++;
        }
    }

    log_printf("TEST 7: Hall Effect Sensor %d\n", (checkCount == 3));
    return (checkCount == 3);
}







