/*
 *  main.c
 *
 *  Created on: 4-Apr-2018
 *
 *  Copyright (c) 2018, Appiko
 *  Author : Tejas Vasekar (https://github.com/tejas-tj)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
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
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HW_TESTING_APP_H
#define HW_TESTING_APP_H

/**
 * @brief This function is used to check power on the board
 * If program execution starts properly, it means power source is connected
 */
uint32_t power_test(void);
/**
 * @brief This fucntion is used to check DC/DC converter. If DC/DC circuit is
 * present at input supply, funtion will return value 1 
 */
uint32_t dc_dc_test(void);
/**
 * @brief This fucntion is used to check on board LEDs. Operator has to be there
 * to check is LEDs are blinking properly or not. 
 */
uint32_t led_test(void);
/**
 * @brief This fucntion is used to check on low frequency crystal. Without 
 * LF-XTAL we cannot use RTC. Therefore RTC is used to detect LF-XTAL
 */
uint32_t crystal_test(void);
/**
 * @brief This fucntion is used to check output RC very low frequency filter.
 * This function will actully check for any AC part present in offset voltage.
 */
uint32_t rc_test(void);
/**
 * @brief This fucntion is used to check output filter designed for f(max)=10Hz
 * This test will detect if any other frequency is supplied to device other
 * than 10 Hz
 */
uint32_t freq_filter_test(void);
/**
 * @brief This fucntion is used to check MCP4012 pot. In this function we will
 * check an ratio between two signals obtained by changing gain by changing pot
 * values. 
 */
uint32_t pot_test(void);
#endif
