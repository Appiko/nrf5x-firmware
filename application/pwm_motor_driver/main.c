/*
 *  main.c : Application to blink LED
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
 * @defgroup hello_world_blinky Hello World Blinky
 * @brief A simple application that blinks a LED.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "nrf_util.h"
#include "simple_pwm.h"
#include "boards.h"
#include "log.h"

#define SPEED_CONTROLLER_PIN 26
#define PWM_MOTOR_PIN 14
#define GPIOTE_CHANNEL_USED 5


#define MAX_COUNT 3000
#define SLOW_COUNT 1000
#define MEDIUM_COUNT 1500
#define FAST_COUNT 3000

typedef enum {
    SLOW,
    MEDIUM,
    FAST,
    TOTAL
} speed;

speed current_speed;

void init() {
    /* Configure pins */
    hal_gpio_cfg_input(SPEED_CONTROLLER_PIN, HAL_GPIO_PULL_UP);
    hal_gpio_cfg_output(PWM_MOTOR_PIN, 1);

    //    Configure GPIO interrupt.
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
            ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
            ((SPEED_CONTROLLER_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);

    // Set to give the interrupt
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN5_Enabled << GPIOTE_INTENSET_IN5_Pos) & GPIOTE_INTENSET_IN5_Msk;


    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);


    current_speed = SLOW;

    hal_gpio_cfg_output(LED_1, LEDS_ACTIVE_STATE);
        hal_gpio_cfg_output(LED_2, !LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_3, !LEDS_ACTIVE_STATE);


    simple_pwm_init(SIMPLE_PWM_TIMER_FREQ_16MHz, MAX_COUNT);
    simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL1, PWM_MOTOR_PIN, SLOW_COUNT);
}

void GPIOTE_IRQHandler() {
    log_printf("Changing state to ");

    // Reset interrupt flag
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;

        simple_pwm_stop();

    switch (current_speed) {
        case SLOW:
            hal_gpio_cfg_output(LED_1, !LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_2, LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_3, !LEDS_ACTIVE_STATE);
            //            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL0, LED_2, (MAX_COUNT - MEDIUM_COUNT));
            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL1, PWM_MOTOR_PIN, MEDIUM_COUNT);

            log_printf("MEDIUM \n");
            break;
        case MEDIUM:
            hal_gpio_cfg_output(LED_1, !LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_2, !LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_3, LEDS_ACTIVE_STATE);
            //            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL0, LED_2, (MAX_COUNT - FAST_COUNT));
            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL1, PWM_MOTOR_PIN, FAST_COUNT);


            log_printf("FAST \n");
            break;
        case FAST:
            hal_gpio_cfg_output(LED_1, LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_2, !LEDS_ACTIVE_STATE);
            hal_gpio_cfg_output(LED_3, !LEDS_ACTIVE_STATE);
//            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL0, LED_2, (MAX_COUNT - SLOW_COUNT));
            simple_pwm_channel_setup(SIMPLE_PWM_CHANNEL1, PWM_MOTOR_PIN, SLOW_COUNT);


            log_printf("SLOW \n");
            break;
        default:
            break;
    }

    current_speed = ((current_speed + 1) % TOTAL);
        simple_pwm_start();
}

void leds_init(void) {
    hal_gpio_cfg_output(LED_1, LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_2, !LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_1, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_2, LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_1, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_2, !LEDS_ACTIVE_STATE);
}

/**
 * @brief Function for the main entry of the application.
 */
int main(void) {


    leds_init();


    log_init();
    log_printf("\nResetting\n");

    log_printf("Initializing pins...\n");
    init();


    simple_pwm_start();
    log_printf("simple PWM started\n");


    while (1) {
    }
}

/** @} */
/** @} */
