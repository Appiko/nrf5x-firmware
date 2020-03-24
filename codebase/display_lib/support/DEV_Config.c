/*****************************************************************************
* | File      	:   DEV_Config.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master
*                and enhance portability
*----------------
* |	This version:   V2.0
* | Date        :   2018-10-30
* | Info        :
# ******************************************************************************
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
 *  DEV_Config.c : SPI Abstraction layer
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
#include "DEV_Config.h"
#include "hal_spim.h"
#include "nrf_util.h"
#include "stddef.h"
#include "log.h"

uint32_t EPD_RST_PIN;
uint32_t EPD_DC_PIN;
uint32_t EPD_CS_PIN;
uint32_t EPD_BUSY_PIN;

void DEV_SPI_WriteByte(UBYTE value)
{
    hal_spim_tx_rx (&value, 1, NULL, 0);
    while (hal_spim_is_busy ());
}


int DEV_Module_Init(DEV_Config_t * init_config)
{
    hal_gpio_cfg_output (init_config->cs_pin, 1);
    hal_gpio_cfg_output (init_config->clk_pin, 0);
    hal_gpio_cfg_output (init_config->din_pin, 0);
    hal_gpio_cfg_input (init_config->dout_pin, HAL_GPIO_PULL_DISABLED);
    
    hal_spim_init_t spim_init = 
    {
        .sck_pin = init_config->clk_pin,
        .csBar_pin = init_config->cs_pin,
        .miso_pin = init_config->dout_pin,
        .mosi_pin = init_config->din_pin,
        .byte_order = HAL_SPIM_MSB_FIRST,
        .freq = HAL_SPIM_FREQ_2M,
        .spi_mode = HAL_SPIM_SPI_MODE0,
        .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
    };
    hal_spim_init (&spim_init);
    
    hal_gpio_cfg_output (init_config->dc_pin, 1);
    hal_gpio_cfg_output (init_config->rst_pin, 1);
    hal_gpio_cfg_input (init_config->busy_pin, HAL_GPIO_PULL_DISABLED);
    
    EPD_BUSY_PIN = init_config->busy_pin;
    EPD_CS_PIN = init_config->cs_pin;
    EPD_DC_PIN = init_config->dc_pin;
    EPD_RST_PIN = init_config->rst_pin;
    
    return 0;
}

void DEV_Module_Exit(void)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    //close 5V
    DEV_Digital_Write(EPD_RST_PIN, 0);
}

