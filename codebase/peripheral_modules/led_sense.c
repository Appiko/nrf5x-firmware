

/*
 *  led_sense.c
 *
 *  Created on: 04-May-2018
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
