/**
 *  senseberx_rev3.h : SenseBe Rev 3 board definitions
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
 * @defgroup board_sensebe_rev3 Appiko Sense Be unit revision 3
 * @brief Board definitions for the 3rd revision of the transceiver unit of SenseBe
 *  active motion sensor by Appiko
 * @{
 */


#ifndef PLATFORM_SENSEBE_REV1_H_
#define PLATFORM_SENSEBE_REV1_H_

/** @anchor sensebe_rev1_gpio
 * @name Pins for testing and debugging
 * @{*/
#define GPIO1_PIN       17
#define GPIO2_PIN       16
/** @} */

/** @anchor sensebe_rev1_leds
 * @name LED definitions for SenseBe 
 * @note LED_1 and LED_2 are defines of the LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED         9
#define LED_GREEN       10

#define LED_1           9
#define LED_2           10

/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE   1
/** @} */

/** @anchor sensebe_rev1_light_sense
 * @name Light Sensor definitions for SenseBe
 * @{
 */
/** Photodiode light sensing */
#define PHOTODIODE_LIGHT_SENSE     28

/** Photodiode Enable Pin */
#define PHOTODIODE_LIGHT_SENSE_EN  29
/** @} */

/** @anchor sensebe_rev1_config_enable
 * @name Definition of configuration mode enable button pin
 * @{
*/
#define BUTTON_PIN          18
/** The logic level at which the Button will set value */
#define BUTTON_ACTIVE_STATE 0
/** @} */

/** @anchor sensebe_rev1_audio_jack
 * @name Definitions of the pin of the audio jack to trigger camera
 * @{*/
#define JACK_CAM_JACK            13
#define JACK_FOCUS_PIN           14
#define JACK_TRIGGER_PIN         12
/** @} */

/** @anchor sensebe_rev1_TSSP4056_receiver
 * @name TSSP4056 IR receiver output and enable pins definitions
 * @{*/
#define TSSP_RX_OUT       8
#define TSSP_RX_EN        7
/** @} */

/** @anchor sensebe_rev1_IR_transmitter
 * @name IR transmitter
 * @{
 */
/** Pin where signal has to be given */
#define IR_TX_LED_EN         30
/** Pin to control the Regulator */
#define IR_TX_REG_EN         27
/** Power control pin 1 */
#define IR_TX_PWR1           5
/** Power control pin 2 */
#define IR_TX_PWR2           6
/** Pin to measure VLED voltage */
#define IR_TX_VLED           4
/** @} */

/// Battery voltage level sensing pin
#define BATT_VOLTAGE_SENSE  31

/** @anchor sensebe_rev1_serial
 * @name Serial port definitions for Sense rev4
 * @{*/
#define RX_PIN_NUMBER       19
#define TX_PIN_NUMBER       20
#define HWFC                false
#define RTS_PIN_NUMBER      23
#define CTS_PIN_NUMBER      24
/** @} */

///Pin that denotes if a board is TX or RX
/// If Pin is high the board is RX, TX if the pin is low
#define SENSEBE_RX_TX_SEL_PIN  11
#define SENSEBE_RX_BOARD       1
#define SENSEBE_TX_BOARD       0

///Bool define if the circuitry is present for the internal DC-DC of nRF52
#define DC_DC_CIRCUITRY     true

///Bool define if a NFC Antenna circuitry is present
#define NFC_CIRCUITRY       false

///Bool define if the 32 kHz crystal is present for the LFCLK
#define LFCLK_XTAL_PRESENT  true

///Bool define if a crystal is present for the HFCLK
#define HFCLK_XTAL_PRESENT  true

/** Low frequency clock source used when initializing the SoftDevice */
#define BOARD_LFCLKSRC_STRUCT  {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                    .rc_ctiv       = 0,                                \
                                    .rc_temp_ctiv  = 0,                                \
                                    .accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM}

#define BOARD_LFCLKSRC         NRF_CLOCK_LFCLK_Xtal

#endif /* PLATFORM_SENSEBE_RX_REV3_H_ */
/**
 * @}
 * @}
 */
