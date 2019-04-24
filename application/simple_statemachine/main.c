/*
 *  main.c : Application to study simple statemachine
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

#define SS_TIME_MS     5052

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
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(SS_TIME_MS), repeat_handler);
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
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(SS_TIME_MS), repeat_handler);
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
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(SS_TIME_MS), repeat_handler);
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
        ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(SS_TIME_MS), repeat_handler);
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
    uart_printf_init(UART_PRINTF_BAUD_1M);
    tfp_printf("Hello World %d!\n", 1);

    hfclk_xtal_init_blocking();
    lfclk_init(LFCLK_SRC_Xtal);

    ms_timer_init(APP_IRQ_PRIORITY_MID);

    hal_gpio_cfg_input(BUTTON_1, HAL_GPIO_PULL_UP);

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
