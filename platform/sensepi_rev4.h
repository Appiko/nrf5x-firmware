/**
 *  sensepi_rev4.h : SensePi Rev 4 board definitions
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
 * @addtogroup group_platform
 * @{
 *
 * @defgroup board_sense_rev4 Appiko SensePi revision 4
 * @brief Board definitions for the 4th revision of the passive motion sensor by Appiko
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
