/*
 *  main.c : Application to print on rtt
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
 * @defgroup hello_world_rtt Hello World RTT printf
 * @brief A simple application that blinks a LED and outputs a printf statement over
 * SEGGER's Real Time Transfer (RTT) channel through the JLink debugger.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "boards.h"
#include "SEGGER_RTT.h"

/**
 * @brief Function for application main entry.
 */
int main(void){
  /* Configure a LED as output. */
  hal_gpio_cfg_output(LED_1, 0);
  /* Initial printf */
  SEGGER_RTT_printf(0, "Hello World over RTT!\n");

  uint32_t count = 0;

  /* Toggle LED 1 and keep printing hello and an
   * incremented integer */
  while (true){
    SEGGER_RTT_printf(0, "Hello %d\n", count);
    hal_gpio_pin_toggle(LED_1);
    hal_nop_delay_ms(1500);
    count++;
  }
}

/** @} */
/** @} */
