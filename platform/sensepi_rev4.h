/*
 *  sense_rev4.h
 *
 *  Created on: 29-Mar-2018
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
 * @defgroup board_sense_rev4 Appiko Sense Revision 4
 * @brief The fourth revision of the animal detector unit by Appiko
 * @{
 */

#ifndef PLATFORM_SENSE_REV4_H_
#define PLATFORM_SENSE_REV4_H_

/** @anchor sense_rev4_leds
 * @name LED definitions for Sense rev4
 * @note LED_1 and LED_2 are defines of the LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED			26
#define LED_GREEN		27

#define LED_1			26
#define LED_2			27

/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE	1
/** The light sensing */
#define LED_LIGHT_SENSE		28
/** @} */

/** @anchor sense_rev4_config_enable
 * @name Definition of configuration mode enable button pin
 * @{ 
*/
#define BUTTON_PIN		25
/** The logic level at which the Button will set value */
#define BUTTON_ACTIVE_STATE	0
/** @} */

/** @anchor sense_rev4_audio_jack
 * @name Definitions of the pin of the audio jack to trigger camera
 * @{*/
#define JACK_AUDIO_JACK_IO_PIN	11
#define JACK_FOCUS_PIN		12
#define JACK_TRIGGER_PIN	13
/** @} */

/// Battery voltage level sensing pin
#define BATT_VOLTAGE_SENSE	31

/** @anchor sense_rev4_serial
 * @name Serial port definitions for Sense rev4
 * @{*/
#define RX_PIN_NUMBER		17
#define TX_PIN_NUMBER		20
#define HWFC			false
#define RTS_PIN_NUMBER		23
#define CTS_PIN_NUMBER		24
/** @} */


/** @anchor sense_rev4_amp_pir
 * @name Pins for the amplified and filtered PIR signal
 * @{*/
/** The amplified and filtered output of the PIR sensor */
#define PIR_AMP_SIGNAL_PIN		4
/** The DC offset of the amplified PIR sensor output */
#define PIR_AMP_OFFSET_PIN		5
/** @} */

/** @anchor sense_rev4_mcp4012
 * @name Pins for driving MCP4012
 * @{*/
/** Chip Select Pin for MCP4012 */
#define MCP4012T_CS_PIN		6
/** U/D pin to set wiper value for MCP4012 */
#define MCP4012T_UD_PIN	7
/** SCK pin required for SPI communication */
#define SPI_SCK_PIN		8
/** @} */

/** @anchor sense_rev4_gpio
 * @name Pins for testing and debugging
 * @{*/
#define GPIO1_PIN		19
#define GPIO2_PIN		18
/** @} */

///Bool define if the circuitry is present for the internal DC-DC of nRF52
#define DC_DC_CIRCUITRY		true

///Bool define if a NFC Antenna circuitry is present
#define NFC_CIRCUITRY		false

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
#endif /* PLATFORM_SENSE_REV4_H_ */
/**
 * @}
 * @}
 */
