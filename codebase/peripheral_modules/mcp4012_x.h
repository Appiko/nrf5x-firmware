/**
 *  mcp4012_x.h : MCP4012 Driver
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
 * @addtogroup group_peripheral_modules
 *
 * @{
 * @defgroup group_mcp4012_x MCP4012 Driver
 * @brief This file contains the declarations required to drive MCP4012 pot. These
 * simple funtion helps user to set wiper value for MCP4012 easily.
 * @{
 */

#ifndef MCP4012_X_H
#define MCP4012_X_H
#include "stdint.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef SPIM_USED_MCP4012_DRIVER 
#define SPIM_USED_MCP4012_DRIVER 0
#endif


/**
 * @brief This function is used to initialize MCP4012 in our program. This will set
 *  wiper value of MCP4012 to zero. Which will be then increased to desired value.
 * @param CS_bar_pin_no This parameter is used to configure that pin number as CS_bar
 * @param UD_bar_pin_no This parameter is used to configure that pin number as UD_bar
 * @param SCK_pin_no This parameter is used to configure that pin number as SCK pin
 * @note SCK pin is not required in our application, but we cannot use SPI without clock pulses. 
 *  Make sure to use pin which is not been used or will be used in future.
 */
void mcp4012_init(uint32_t CS_bar_pin_no, uint32_t UD_bar_pin_no, uint32_t SCK_pin_no);


/**
 * @brief This function is used to set the value of MCP4012 wiper value
 * @param value_sent_by_user This parameter is used to set wiper value  This value is
 *  transfered to MCP4012 using simple SPI which is used as Two Wire Protocol over here.
 */
void mcp4012_set_value(uint32_t value_sent_by_user);
/**
 * @} *
 * @} *
 */

#endif /*MCP4012_X_H*/
