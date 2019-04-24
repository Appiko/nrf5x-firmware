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
#include "boards.h"

/**
 * @brief Function for the main entry of the application.
 */
int main(void){
  /* Configure the LED 1 as output. */
  hal_gpio_cfg_output(LED_1, 1);

  /* Toggle LED 1 after every 500 ms */
  while (true){
    hal_gpio_pin_toggle(LED_1);
    hal_nop_delay_ms(500);
  }
}

/** @} */
/** @} */
