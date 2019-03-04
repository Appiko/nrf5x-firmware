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
 * @defgroup bluey_demo An application to demo the Bluey's capabilities
 * @brief A Bluey demo
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include "math.h"

#include "nrf.h"

#include "boards.h"
#include "hal_clocks.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "hal_twim.h"
#include "common_util.h"
#include "log.h"
#include "nrf_util.h"
#include "nrf_sdm.h"
#include "ble_adv.h"
#include "ble.h"
#include "ms_timer.h"
#include "LSM6DS3.h"
#include "lsm_testing_ble.h"
#include "hal_pin_analog_input.h"
#include "simple_adc.h"

#define SMOKE_DETECT_PIN 2

#define ANI_SMOKE_PIN PIN_TO_ANALOG_INPUT(SMOKE_DETECT_PIN)

int16_t x_data, y_data, z_data;

static uint32_t my_sqrt (uint32_t num)
{
    float xn, xm, flag, temp;
    xn = 0;
    xm = 800;
    flag = 0;
    while (flag == 0)
    {
        xn = (xm + (num / xm)) / 2;
        if (xn == xm)
        {
            flag = 1;
        }
        else
        {
            xm = xn;
        }
    }
    temp = xn * 1000;
    return (uint32_t) temp;
}

bool compare_percent (uint32_t data, uint32_t ref, float per)
{
    if ((ref - (ref * per / 100)) <= data && data <= (ref + (ref * per / 100)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint32_t avg_uint32_x100 (uint32_t * p_arr, uint32_t no_of_entries)
{
    uint32_t add = 0;
    float avg = 0;
    for(uint32_t cnt = 0; cnt < no_of_entries; cnt++)
    {
        add += p_arr[cnt];
    }
    avg = add/no_of_entries;
    return (uint32_t)(avg * 100);
}

static mod_ble_data_t ble_data;

void lsm_100ms_handler ()
{
    
    LSM6DS3_read_accl_data (&x_data, &y_data, &z_data);
    x_data = (LSM6DS3_accelData_in_g(x_data));
    x_data = (x_data * ((x_data > 0) - (x_data < 0)));
    y_data = (LSM6DS3_accelData_in_g(y_data) );
    y_data = (y_data * ((y_data > 0) - (y_data < 0)));
    z_data = (LSM6DS3_accelData_in_g(z_data) );
    z_data = (z_data * ((z_data > 0) - (z_data < 0)));
}

void ms_timer_handler ()
{
    static int32_t arr_diff_acce[10];
    static uint32_t curr_res_acce = 0;
    static uint32_t prev_res_acce = 0;
    static uint32_t res_acce_loc = 0;
    static uint32_t curr_avg_res_acce_x100 = 0;
    static uint32_t arr_smoke_val[10];
    static uint32_t curr_avg_smoke_val_x100 = 0;
    lsm_100ms_handler ();
    curr_res_acce = my_sqrt ((x_data * x_data) + (y_data * y_data) + (z_data * z_data))/1000;
    arr_diff_acce[res_acce_loc] = curr_res_acce - prev_res_acce;
    if(arr_diff_acce[res_acce_loc] < 0)
    {
        arr_diff_acce[res_acce_loc] = 0 - arr_diff_acce[res_acce_loc];
    }
    arr_smoke_val[res_acce_loc] = simple_adc_get_value (SIMPLE_ADC_GAIN1_6, ANI_SMOKE_PIN);
    prev_res_acce = curr_res_acce;
        curr_avg_res_acce_x100 = avg_uint32_x100 ((uint32_t *)arr_diff_acce, 10);
        ble_data.avg_acce = curr_avg_res_acce_x100 / 100;
    if(res_acce_loc == 9)
    {   
        curr_avg_smoke_val_x100 = avg_uint32_x100 (arr_smoke_val, 10);
        ble_data.avg_smoke = curr_avg_smoke_val_x100 / 100;
    }
        lsm_ble_update_status_byte (&ble_data);
    res_acce_loc = (res_acce_loc + 1) % 10;
    
    
    
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
    hal_gpio_pin_write(LED_RED, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

void on_connect ()
{
    
}
/**
 * Different calls to sleep depending on the status of Softdevice
 */
void slumber(void)
{
    uint8_t is_sd_enabled;
    sd_softdevice_is_enabled(&is_sd_enabled);
    // Would in the SENSING mode
    if(is_sd_enabled == 0)
    {
        __WFI();
    }
    else
    {
        sd_app_evt_wait();
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    log_init();
    log_printf("Hello World from LSM6D..!!\n");

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    lfclk_init (LFCLK_SRC_Xtal);
    LSM6DS3_init();
    {
        lsm_ble_stack_init ();
        lsm_ble_gap_params_init ();
        lsm_ble_adv_init ();
        lsm_ble_adv_start (on_connect);
        lsm_ble_service_init ();
    }
    ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(100), ms_timer_handler);
    while (true)
    {
        slumber ();
    }
}

/** @} */
/** @} */
