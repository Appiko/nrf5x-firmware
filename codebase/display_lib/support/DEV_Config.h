/*****************************************************************************
* | File      	:   DEV_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
* 1.add:
*   UBYTE\UWORD\UDOUBLE
* 2.Change:
*   EPD_RST -> EPD_RST_PIN
*   EPD_DC -> EPD_DC_PIN
*   EPD_CS -> EPD_CS_PIN
*   EPD_BUSY -> EPD_BUSY_PIN
* 3.Remote:
*   EPD_RST_1\EPD_RST_0
*   EPD_DC_1\EPD_DC_0
*   EPD_CS_1\EPD_CS_0
*   EPD_BUSY_1\EPD_BUSY_0
* 3.add:
*   #define DEV_Digital_Write(_pin, _value) bcm2835_gpio_write(_pin, _value)
*   #define DEV_Digital_Read(_pin) bcm2835_gpio_lev(_pin)
*   #define DEV_SPI_WriteByte(__value) bcm2835_spi_transfer(__value)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
/**
 *  DEV_Config.h : SPI Abstraction layer
 *  Copyright (C) 2020  Appiko
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include "hal_gpio.h"
#include "hal_nop_delay.h"
#include <stdint.h>

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * e-Paper GPIO
**/
extern uint32_t EPD_RST_PIN;
extern uint32_t EPD_DC_PIN;
extern uint32_t EPD_CS_PIN;
extern uint32_t EPD_BUSY_PIN;

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) hal_gpio_pin_write(_pin, _value)
#define DEV_Digital_Read(_pin) hal_gpio_pin_read(_pin)


typedef struct 
{
    uint32_t busy_pin;
    uint32_t rst_pin;
    uint32_t dc_pin;
    uint32_t cs_pin;
    uint32_t clk_pin;
    uint32_t din_pin;
    uint32_t dout_pin; //not used
}DEV_Config_t;
/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms) hal_nop_delay_ms(__xms);

void DEV_SPI_WriteByte(UBYTE value);

int DEV_Module_Init(DEV_Config_t * init_config);
void DEV_Module_Exit(void);
#endif
