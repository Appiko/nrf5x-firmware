/**
 *  led_sense.c : Light sensing using LED
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

#include "led_sense.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "simple_adc.h"
#include "hal_gpio.h"

static uint32_t led_pin, sense_pin, off_val;

void led_sense_init(uint32_t led_out_pin, uint32_t led_sense_analog_pin,
        uint32_t led_off_val)
{
    ASSERT(led_pin < 32);
    ASSERT(led_sense_analog_pin < SAADC_CH_PSELP_PSELP_VDD);

    led_pin = led_out_pin;
    off_val = led_off_val;
    sense_pin = led_sense_analog_pin;
}

void led_sense_cfg_input(bool is_input_on)
{
    if(is_input_on)
    {
        hal_gpio_cfg(led_pin,
            GPIO_PIN_CNF_DIR_Input,
            GPIO_PIN_CNF_INPUT_Disconnect,
            GPIO_PIN_CNF_PULL_Disabled,
            GPIO_PIN_CNF_DRIVE_S0S1,
            GPIO_PIN_CNF_SENSE_Disabled);
    }
    else
    {
        hal_gpio_cfg_output(led_pin, off_val);
    }
}

uint32_t led_sense_get(void)
{
    return simple_adc_get_value(SIMPLE_ADC_GAIN1_6, sense_pin);
}
