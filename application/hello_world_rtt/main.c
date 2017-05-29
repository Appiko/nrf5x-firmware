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
