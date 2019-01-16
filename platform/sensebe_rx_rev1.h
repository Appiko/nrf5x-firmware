/*
 *  sensebe_rx_rev1.h
 *
 *  Created on: 22-Oct-2018
 *
 *  Copyright (c) 2018, Appiko
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
