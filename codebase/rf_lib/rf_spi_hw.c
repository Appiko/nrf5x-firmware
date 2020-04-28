/*
 *  rf_spi_hw.c : <Write brief>
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

#include "hal_spim.h"
#include "rf_spi_hw.h"
#include "hal_gpio.h"

uint32_t rf_spi_init (rf_spi_init_t * p_spi_init)
{
    hal_gpio_cfg_output (p_spi_init->csn_pin, 1);
    hal_gpio_cfg_output (p_spi_init->sclk_pin, 0);
    hal_gpio_cfg_input (p_spi_init->miso_pin, HAL_GPIO_PULL_DISABLED);
    hal_gpio_cfg_output (p_spi_init->mosi_pin, 0);
    hal_spim_init_t spim_init =
    {
        .csBar_pin = p_spi_init->csn_pin,
        .sck_pin = p_spi_init->sclk_pin,
        .miso_pin = p_spi_init->miso_pin,
        .mosi_pin = p_spi_init->mosi_pin,
        .spi_mode = HAL_SPIM_SPI_MODE0,
        .byte_order = HAL_SPIM_MSB_FIRST,
        .freq = HAL_SPIM_FREQ_125K,
        .irq_priority = p_spi_init->irq_priority
    };
    hal_spim_init (&spim_init);
    return 0;
}
