/*
 *  Created on: 08-July-2017
 *
 *  Copyright (c) 2017, Appiko
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
 * @defgroup simple_statemachine A demo of a simple state machine
 * @brief A simple state machine of three states is implemented in this example
 * both switch statement and function pointers. This example is compiled for the
 * Bluey board. The state machine is as shown in the diagram below. Note that
 * `button` in the diagram means the release of the button.
 * @{
 *
 * @dot
 * digraph State_machine_diagram {
 *  rankdir="LR";
 *  no [shape = circle, label = "No light\n(not timeout)"]
 *  rd [shape = circle, label ="Red LED on"]
 *  gn [shape = circle, label ="Green LED on"]
 *  no -> rd [label = "button"];
 *  rd -> no [label = "5 sec timeout"];
 *  rd -> gn [label = "button"];
 *  gn -> rd [label = "5 sec timeout"];
 *  gn -> gn [label = "button/\nreset timeout"];
 * }
 * @enddot
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "boards.h"
#include "hal_clocks.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "common_util.h"
#include "tinyprintf.h"
#include "uart_printf.h"
#include "ms_timer.h"
#include "nrf_util.h"

/** @brief The three states of the state machine */
typedef enum
{
    OFF_STATE, RED_STATE, GREEN_STATE
} sm_states;
sm_states state = OFF_STATE;

/** @brief The type of function pointer that'll be used by the
 * processing functions of the three states
 */
typedef void (*state_process)(void);

/**
 * @brief 5s timeout handler used to go from green to red state or
 *  red to off state.
 */
void repeat_handler(void)
{
    switch (state)
    {
    case RED_STATE:
        state = OFF_STATE;
        hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_BLUE, !LEDS_ACTIVE_STATE);
        tfp_printf("RED to OFF\n");
        break;
    case GREEN_STATE:
        state = RED_STATE;
        hal_gpio_pin_write(LED_RED, LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_BLUE, !LEDS_ACTIVE_STATE);
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, RTC_TICKS_MS(5000), repeat_handler);
        tfp_printf("GREEN to RED\n");
        break;
    case OFF_STATE:
    default:
        break;
    }
}

/**
 * @brief Function that returns if the button was released from the previous time
 *  it was called
 * @return True if a button release happened, else false
 */
bool button_check(void)
{
    static uint32_t prev_state = !BUTTONS_ACTIVE_STATE;
    bool falling_edge;
    uint32_t curr_state = hal_gpio_pin_read(BUTTON_1);
    if ((curr_state == !BUTTONS_ACTIVE_STATE) && (prev_state == BUTTONS_ACTIVE_STATE))
    {
        falling_edge = true;
    }
    else
    {
        falling_edge = false;
    }
    prev_state = curr_state;
    return falling_edge;
}

/** @brief Green state process checks if there is a button press to reset the timer */
void green_process(void)
{
    bool btn_check = button_check();
    if (btn_check)
    {
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, RTC_TICKS_MS(5000), repeat_handler);
    }
}

/** @brief Off state process checks if there is a button press to go to red state */
void off_process(void)
{
    bool btn_check = button_check();
    if (btn_check)
    {
        hal_gpio_pin_write(LED_RED, LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_BLUE, !LEDS_ACTIVE_STATE);
        state = RED_STATE;
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, RTC_TICKS_MS(5000), repeat_handler);
        tfp_printf("OFF to RED\n");
    }
}

/** @brief Red state process checks if there is a button press to go to green state */
void red_process(void)
{
    bool btn_check = button_check();
    if (btn_check)
    {
        hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_GREEN, LEDS_ACTIVE_STATE);
        hal_gpio_pin_write(LED_BLUE, !LEDS_ACTIVE_STATE);
        state = GREEN_STATE;
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, RTC_TICKS_MS(5000), repeat_handler);
        tfp_printf("RED to GREEN\n");
    }
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Cycle through the red, green and blue LEDs */
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

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    uart_printf_init(UART_PRINTF_BAUD_9600);
    tfp_printf("Hello World %d!\n", 1);

    hfclk_xtal_init_blocking();
    lfclk_init(LFCLK_SRC_Xtal);

    ms_timer_init(APP_IRQ_PRIORITY_MID);

    hal_gpio_cfg_input(BUTTON_1, HAL_GPIO_PULL_DISABLED);

    state_process process[3] =
    { &off_process, &red_process, &green_process };

    while (true)
    {
        process[state]();
        //To take care of button debouncing
        hal_nop_delay_ms(50);
    }
}

/** @} */
/** @} */
