/**
 *  sensebe_rx_rev2.h : SenseBe RX Rev 2 board definitions
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
 * @defgroup board_sensebe_rx_rev1 Appiko Sense Be Rx unit revision 1
 * @brief Board definitions for the 1st revision of the receiver unit of SenseBe
 *  active motion sensor by Appiko
 * @{
 */


#ifndef PLATFORM_SENSEBE_RX_REV1_H_
#define PLATFORM_SENSEBE_RX_REV1_H_

/** @anchor sensebe_rx_rev1_gpio
 * @name Pins for testing and debugging
 * @{*/
#define GPIO1_PIN       19
#define GPIO2_PIN       18
/** @} */

/** @anchor sensebe_rx_rev1_leds
 * @name LED definitions for SenseBe Rx rev1
 * @note LED_1 and LED_2 are defines of the LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED         26
#define LED_GREEN       25

#define LED_1           26
#define LED_2           25

/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE   1
/** @} */

/** Photodiode light sensing */
#define PHOTODIODE_LIGHT_SENSE     5

/** Photodiode Enable Pin */
#define PHOTODIODE_ENABLE_PIN GPIO2_PIN

/** @anchor sensebe_rx_rev1_config_enable
 * @name Definition of configuration mode enable button pin
 * @{
*/
#define BUTTON_PIN          8
/** The logic level at which the Button will set value */
#define BUTTON_ACTIVE_STATE 0
/** @} */

/** @anchor sensebe_rx_rev1_audio_jack
 * @name Definitions of the pin of the audio jack to trigger camera
 * @{*/
#define JACK_INSERTED_PIN        11
#define JACK_FOCUS_PIN           12
#define JACK_TRIGGER_PIN         13
/** @} */

/** @anchor sensebe_rx_rev1_txrx_serial
 * @name Serial port definitions for communicating with SenseBe Tx unit
 *  A audio jack connected is used as the port for communication
 * @{*/
#define RXTX_UART_RX_PIN            15
#define RXTX_UART_TX_PIN            16
#define RXTX_UART_JACK_INSERTED     14
/** @} */

/** @anchor sensebe_rx_rev1_TSSP4056_receiver
 * @name TSSP4056 IR receiver output and enable pins definitions
 * @{*/
#define TSSP_RX_OUT       6
#define TSSP_RX_EN        7
/** @} */

/// Battery voltage level sensing pin
#define BATT_VOLTAGE_SENSE  31

/** @anchor sensebe_rx_rev1_serial
 * @name Serial port definitions for Sense rev4
 * @{*/
#define RX_PIN_NUMBER       17
#define TX_PIN_NUMBER       20
#define HWFC                false
#define RTS_PIN_NUMBER      23
#define CTS_PIN_NUMBER      24
/** @} */

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

#endif /* PLATFORM_SENSEBE_RX_REV1_H_ */
/**
 * @}
 * @}
 */
