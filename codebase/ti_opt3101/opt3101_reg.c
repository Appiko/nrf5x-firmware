/**
 *  opt3101_reg.c : OPT3101 Register manipulation driver.
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "opt3101_reg.h"
#include "hal_twim.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"
#include "log.h"

void opt3101_hw_connect (opt3101_hw_conf_t * i2c_config)
{
    hal_gpio_cfg_output (i2c_config->cs_bar_pin,1);
    hal_twim_init_config_t twi_init = 
    {
        .scl = i2c_config->sck_pin,
        .sda = i2c_config->sda_pin,
        .irq_priority = i2c_config->irq_priority,
        .address = 88,
        .frequency = HAL_TWI_FREQ_400K,
    };
    hal_twim_init (&twi_init);
    hal_gpio_pin_clear (i2c_config->cs_bar_pin);
    hal_nop_delay_us(10);
    hal_gpio_pin_set (i2c_config->cs_bar_pin);
    log_printf("%s : %d\n", __func__, hal_twim_get_current_adrs ());
}

uint8_t opt3101_reg_modify (uint8_t addr, uint8_t lsb_loc, uint8_t msb_loc, uint32_t val)
{
    uint8_t status;
    uint32_t temp_reg = 0;
    uint8_t buff[4];
    buff[0] = addr;
    status = hal_twim_tx_rx ((uint8_t * )&addr, 1, (uint8_t * )&temp_reg, 3);
    hal_nop_delay_us(200);
    
    temp_reg = (( 0xFFFFFF >> (23 - msb_loc)) & (val << lsb_loc)) | 
        (temp_reg & ~((0xFFFFFF<<lsb_loc) & (0xFFFFFF >> (23-msb_loc)))); 

    memcpy (&buff[1], &temp_reg, 3);
    status = hal_twim_tx (buff, 4);
    hal_nop_delay_ms(20);

    return status;
}

uint8_t opt3101_reg_read (uint8_t addr, uint8_t lsb_loc, uint8_t msb_loc, uint32_t * val)
{
    uint8_t status;

    uint32_t temp_reg = 0;
    status = hal_twim_tx_rx ((uint8_t * )&addr, 1, (uint8_t * )&temp_reg, 3);
    hal_nop_delay_us(200);

    *val = (uint32_t)((0xFFFFFF >> (23 - msb_loc)) & (temp_reg) & (0xFFFFFF << lsb_loc));
    return status;
}
