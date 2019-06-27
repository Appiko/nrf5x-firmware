/**
 *  opt3101_reg.h : OPT3101 Register manipulation driver.
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


#ifndef OPT3101_REG_H
#define OPT3101_REG_H

/**
 * @addtogroup group_ti_opt3101
 * @{
 *
 * @defgroup group_opt3101_reg OPT3101 Register manipulation
 * @brief Abstraction layer between OPT3101 driver and host's I2C driver
 *
 * @{
 */

#include "stdint.h"
#include "nrf_util.h"

/** Structure to store information required for I2C connection with  */
typedef struct
{
    /** Pin number of SCK/SCL pin */
    uint32_t sck_pin;
    /** Pin number of SDA pin */
    uint32_t sda_pin;
    /** Pin number of CS pin : not a part of I2C but required to connect with OPT3101*/
    uint32_t cs_bar_pin;
    /** IRQ Priority : May differ in different platform */
    app_irq_priority_t irq_priority;
}opt3101_hw_conf_t;

/**
 * @brief Function to connect OPT3101 with host hardware
 * @param i2c_config Structure pointer of data type @ref opt3101_hw_conf_t
 * @return I2C status 
 * @retval 0 Successful
 */
void opt3101_hw_connect (opt3101_hw_conf_t * i2c_config);

/**
 * @brief Function to modify a particular field from particular 24bit register
 * @param addr Address of register
 * @param lsb_loc Starting bit number of said field
 * @param msb_loc Ending bit number of said field
 * @param val Value which is to be modified
 * @return I2C status 
 * @retval 0 Successful
 */
uint8_t opt3101_reg_modify (uint8_t addr, uint8_t lsb_loc, uint8_t msb_loc, uint32_t val);

/**
 * @brief Function to read certain field from register and store it in buffer
 * @param addr Address of register
 * @param lsb_loc Staring bit number of said fields
 * @param msb_loc Ending bit number of said field
 * @param buff Buffer where read values are to be stored
 * @return I2C status 
 * @retval 0 Successful
 */
uint8_t opt3101_reg_read (uint8_t addr, uint8_t lsb_loc, uint8_t msb_loc, uint32_t * val);

/**
 * @}
 * @}
 */

#endif //OPT3101_REG_H
