/*
 *  mcp4012_x.h
 *  Creadted on: 28-Mar-2018
 *  Copyright (c) 2018 Appiko
 *  File Author: Tejas Vasekar (https://github.com/tejas-tj)
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or other 
 *  materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may
 *  be used to endorse or promote products derived from this software without specific
 *  prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
 *  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 *  DAMAGE.
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

#if MAIN_H_PRESENT == 1
#include "main.h"
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
