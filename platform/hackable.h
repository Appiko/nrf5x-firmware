/*
 *  sense_rev4.h
 *
 *  Created on: 20-Apr-2018
 *
 *  Copyright (c) 2018, Appiko
 *  Author : Tejas Vasekar (https://github.com/tejas-tj)
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
 * @addtogroup group_platform
 * @{
 *
 * @defgroup board_hackable hackaBLE
 * @brief The hackaBLE board for development and testing.
 * https://github.com/electronut/ElectronutLabs-hackaBLE
 * @{
 */

#ifndef PLATFORM_HACKABLE_H_
#define PLATFORM_HACKABLE_H_

/** @anchor hackable_leds
 * @name LED definitions for hackable
 * @note LED_1, LED_2 and LED_3 are defines of the LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED			19
#define LED_GREEN		20
#define LED_BLUE        17

#define LED_1			19
#define LED_2			20
#define LED_3           17
/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE	0
/** @} */

/** @anchor hackable_serial
 * @name Serial port definitions for Hackable
 * @{*/
#define RX_PIN_NUMBER		25
#define TX_PIN_NUMBER		26
#define HWFC			false
#define RTS_PIN_NUMBER		23
#define CTS_PIN_NUMBER		24
/** @} */

/** @anchor nfc_antenna_connector
  * @name Pin definations for NFC antenna connectors
  * @{*/
#define NFC_1_ANTENNA_CONNECTOR_PIN_NO  9
#define NFC_2_ANTENNA_CONNECTOR_PIN_NO  10
/** @} */

///Bool define if the circuitry is present for the internal DC-DC of nRF52
#define DC_DC_CIRCUITRY		true

///Bool define if a NFC Antenna circuitry is present
#define NFC_CIRCUITRY		true

///Bool define if the 32 kHz crystal is present for the LFCLK
#define LFCLK_XTAL_PRESENT	true

///Bool define if a crystal is present for the HFCLK
#define HFCLK_XTAL_PRESENT	true

/** Low frequency clock source used when initializing the SoftDevice */
#define BOARD_LFCLKSRC_STRUCT  {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                    .rc_ctiv       = 0,                                \
                                    .rc_temp_ctiv  = 0,                                \
                                    .accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM}

#define BOARD_LFCLKSRC         NRF_CLOCK_LFCLK_Xtal
#endif /* PLATFORM_HACKABLE_H_ */
/**
 * @}
 * @}
 */
