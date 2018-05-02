/*
 *  main.c
 *
 *  Created on: 4-Apr-2018
 *
 *  Copyright (c) 2018, Appiko
 *  Author : Tejas Vasekar (https://github.com/tejas-tj)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
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

#include "pir_sense.h"
#include "ms_timer.h"
#include "profiler_timer.h"
#include "simple_adc.h"
#include "mcp4012_x.h"
#include "uart_printf.h"
#include "tinyprintf.h"
#include "mcp4012_x.h"
#include "log.h"

#include "hw_testing_app.h"


#define TIMER_INIT_VALUE (read_time_us())
#define CALC_TIMER_VAL 1000000
#define MAX_FREQ 10

/// ADC value for Vdd/2
#define CALC_OFFSET simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(BATT_VOLTAGE_SENSE))/2

/**
 * @brief This funtion is used to calculate the square root of the number passed
 * using Babylonian method
 * @param num: The number of which square root is to be calculated.
 */
static uint32_t my_sqrt(uint32_t num);

/**
 * @brief This function is used to check if one number is in some percent window
 * of other number
 * @param data The variable which is to be checked in window
 * @param ref Window has to be around this number
 * @param per Percentage value for which window is to be built
 */
static uint32_t compare_percent(uint32_t data, uint32_t ref, float per);

/**
 * @brief This function is used to calculate the rms value for PIR amplified 
 * signal.
 * @return This funtion returns the value which is equal to (int)(rms*1000)
 */
static uint32_t hw_test_obs_sig();

/**
 * @brief This function is used to calculate the frequency for PIR amplified
 * signal.
 * @return This funtions return the value of frequency of signal.
 * @note This function is useful for a signal containing single sine wave with
 * frequency >= 1Hz
 */
static uint32_t hw_test_obs_freq();

/** The possible sources for Low frequency clock's 32 kHz input
 */
typedef enum {
    MY_LFCLK_SRC_RC    = CLOCK_LFCLKSRC_SRC_RC,   ///< Internal RC oscillator.
    MY_LFCLK_SRC_Xtal  = CLOCK_LFCLKSRC_SRC_Xtal, ///< External crystal.
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

uint32_t compare_percent(uint32_t data, uint32_t ref, float per)
{
    if((ref-(ref*per/100))<=data && data<=(ref+(ref*per/100)))
    {
        return 1;
    }
    else
    {
        return 0;    
    }
}

static uint32_t my_sqrt(uint32_t num)
{
    float xn,xm,flag, temp;
    xn = 0; xm = 800; flag = 0;
    while(flag == 0)
    {
        xn = (xm + (num/xm))/2;
        if(xn == xm)
        {
            flag = 1;
        }
        else
        {
            xm = xn;
        }
    }
    temp = xn * 1000;
    return  (uint32_t) temp;
}


//Funtions for actual tests.
// Power Test
uint32_t power_test(void)
{
    hal_gpio_pin_set(LED_RED);
    log_printf("Power_test : 1\n");
    hal_nop_delay_ms(100);
    hal_gpio_pin_clear(LED_RED);
    return 1;
}
// DC/DC Test 
uint32_t dc_dc_test(void)
{
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;
    if(NRF_POWER->DCDCEN == (POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos))
    {
        log_printf("DC_DC_test : 1\n");
        return 1;    
    }
    else
    {
        log_printf("DC_DC_test : 0\n");
        return 0;    
    }
}
// LED test 
uint32_t led_test(void)
{
    hal_gpio_pin_set(LED_GREEN);
    hal_nop_delay_ms(1000);
    hal_gpio_pin_clear(LED_GREEN);
    hal_gpio_pin_set(LED_RED);
    hal_nop_delay_ms(1000);    
    hal_gpio_pin_clear(LED_RED);
    log_printf("LED_test : 1\n");
    return 1;
}

//Crystal test 
/**
 * @note 1 tick for MS_TIMER0 is 31 us long. And all the initialization takes 247 us.
 */

static uint32_t my_lfclk_init(my_lfclk_src_t my_lfclk_src)
{
    uint32_t my_xtal_flag = 0;
    if((NRF_CLOCK->LFCLKSTAT &  //Clock is running
       (CLOCK_LFCLKSTAT_STATE_Running << CLOCK_LFCLKSTAT_STATE_Pos)) &&
       //Correct source is already set
      ((NRF_CLOCK->LFCLKSTAT & CLOCK_LFCLKSTAT_SRC_Msk) == my_lfclk_src))
    {
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
    (void)NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    hal_nop_delay_ms(500);
    /* Wait for the external oscillator to start up. */
    if (NRF_CLOCK->EVENTS_LFCLKSTARTED == 1)
    {
        my_xtal_flag = 1;
    }
    /* Clear the event and the pending interrupt */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    (void)NRF_CLOCK->EVENTS_LFCLKSTARTED;
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
    NRF_CLOCK->INTENCLR = CLOCK_INTENCLR_LFCLKSTARTED_Msk;

    return my_xtal_flag;
 
}

uint32_t crystal_test()
{
    uint32_t flag = 0;
    uint32_t check_status_xtal = 0;
    ms_timer_init(1);
    check_status_xtal = my_lfclk_init(MY_LFCLK_SRC_Xtal);
    ms_timer_start(MS_TIMER0,MS_SINGLE_CALL,32768,capture_lfclk_test);
    profiler_timer_init();
    hal_nop_delay_ms(1020);
    flag = compare_percent(timer_val_us, CALC_TIMER_VAL, 1);
    ms_timer_stop(1);   
    if (check_status_xtal)
    {
        log_printf("Crystal : 1\n");
    }
    else
    {
        log_printf("Crystal : 0\n");
    }
    if (flag && check_status_xtal)
    {
        log_printf("Crystal_test : 1\n");
        return 1;
    }
    else
    {
        log_printf("Crystal_test : 0\n");
        return 0;
    }

}

void capture_lfclk_test (void)
{
    timer_val_us = read_time_us();
    log_printf("Timer_XTAL : %d\n", timer_val_us);
    log_printf("Timer_TIMER : %d\n", CALC_TIMER_VAL);
    profiler_timer_deinit();
}


// RC test 
uint32_t rc_test(void)
{
    uint32_t obs_offset = 0, avg_offset = 0, i = 0, temp_sum = 0;
    mcp4012_set_value(1);
    profiler_timer_init();
    while(read_time_us() < 1200000)
    {
        obs_offset =  simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN));
        temp_sum = temp_sum + obs_offset;
        i++;
    }
    avg_offset = temp_sum / i;
    profiler_timer_deinit();
    log_printf("CALC_OFFSET : %lu\n", CALC_OFFSET);
    log_printf("Avg_Offset : %lu\n", avg_offset);
    if(compare_percent(avg_offset, CALC_OFFSET, 5))
    {
        log_printf("RC_test : 1\n");
        return 1;
    }
    else
    {
        log_printf("RC_test : 0\n");
        return 0;    
    }
}

// Freq filter test 
uint32_t freq_filter_test(void)
{
    uint32_t obs_freq = hw_test_obs_freq();    
    if(obs_freq == MAX_FREQ)
    {
        log_printf("Freq_filter_test : 1\n");
        return 1;
    }
    else    
    {
        log_printf("Freq_filter_test : 0\n");
        return 0;
    }
}

static uint32_t hw_test_obs_freq()
{
    uint32_t temp_signal = 0, cnt_cross = 0, freq = 0;
    const uint32_t temp_offset = CALC_OFFSET;
    do{
        temp_signal = simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN));

    }while(!(compare_percent(temp_signal, temp_offset, 0.1) &&((int)(temp_signal - temp_offset) <= 0)));
    profiler_timer_init();
    while(read_time_us() < 1010000)
    {
        temp_signal = simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN));

        if(compare_percent(temp_offset,temp_signal,0.2))
        {
            cnt_cross++;
            hal_nop_delay_ms(5);
        }
    }
    profiler_timer_deinit();
    freq = cnt_cross/2;
    log_printf("Frequency : %d\n", freq);
    return freq;
}

// POT test
uint32_t pot_test(void)
{
    uint32_t read1 = 0;
    uint32_t read2 = 0;

    mcp4012_set_value(45);
    hal_nop_delay_ms(10);
    read1 = hw_test_obs_sig();
    log_printf("POT_test_Read1 : %d\n", read1);

    mcp4012_set_value(0);
    hal_nop_delay_ms(10);
    read2 = hw_test_obs_sig();
    log_printf("POT_test_Read2 : %d\n", read2);
    log_printf("ratio_Read1_Read2 : %d\n", ((read1*1000)/read2));

    if(!compare_percent(read1, read2, 100))
    {
        log_printf("POT_test : 1\n");
        return 1;
    }
    else
    {
        log_printf("POT test : 0\n");
        return 0;    
    }
}

static uint32_t hw_test_obs_sig()
{
    uint32_t temp_signal, avg_signal, rms, i;
    uint64_t temp_sum;
    temp_signal = 0; avg_signal = 0; temp_sum = 0, i = 0;
    const uint32_t temp_offset = CALC_OFFSET;
    do{
        temp_signal = simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN));
    }while((!compare_percent(temp_signal, temp_offset, 0.1)) && (((int)(temp_signal - temp_offset)) > 0));
    profiler_timer_init();
    while(read_time_us() < 1000000)
    {
        temp_signal = simple_adc_get_value(SIMPLE_ADC_GAIN1_5, PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN));
        temp_sum = (temp_sum + (((int)(temp_signal-temp_offset))*((int)(temp_signal-temp_offset))));
        i++;
    }
    profiler_timer_deinit();
    avg_signal = temp_sum / i;
    rms = my_sqrt(avg_signal);
    return rms;
}


