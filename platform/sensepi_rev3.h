/**
 *  sensepi_rev3.h : SensePi Rev 3 board definitions
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
 * @defgroup board_sense_rev3 Appiko Sense Revision 3
 * @brief The third revision of the animal detector unit by Appiko
 * @{
 */

#ifndef PLATFORM_SENSE_REV3_H_
#define PLATFORM_SENSE_REV3_H_

/** @anchor sense_rev3_leds
 * @name RGB LED definitions for Sense rev3
 * @note LED_1, LED_2 and LED_3 are defines of the RGB LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED        28
#define LED_GREEN      29
#define LED_BLUE       27

#define LED_1          28
#define LED_2          29
#define LED_3          27
/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE 1
/** The light sensing pin with the Green LED */
#define LED_LIGHT_SENSE 30
/** @} */

/** @anchor sense_rev3_config_enable
 * @name Definition of configuration mode enable button pin
 * @{
*/
#define BUTTON_PIN      26
/** The logic level at which the Button will set value */
#define BUTTON_ACTIVE_STATE 0
/** @} */

/** @anchor sense_rev3_audio_jack
 * @name Definitions of the pin of the audio jack to trigger camera
 * @{*/
#define JACK_DETECT        13
#define JACK_FOCUS         12
#define JACK_TRIGGER      11
/** @} */

/// Battery voltage level sensing pin
#define BATT_VOLTAGE_SENSE  31

/** @anchor sense_rev3_serial
 * @name Serial port definitions for Sense rev3
 * @{*/
#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  17
#define CTS_PIN_NUMBER 18
#define RTS_PIN_NUMBER 20
#define HWFC false
/** @} */

/** @anchor sense_rev3_rc_pir
 * @name Pins for the signal directly from the PIR through a RC filter
 * @{*/
/** The RC filtered output of the PIR sensor */
#define PIR_RC_SIGNAL_PIN  4
/** The DC offset of the PIR sensor output */
#define PIR_RC_OFFSET_PIN  5
/** @} */

/** @anchor sense_rev3_amp_pir
 * @name Pins for the amplified and filtered PIR signal
 * @{*/
/** The amplified and filtered output of the PIR sensor */
#define PIR_AMP_SIGNAL_PIN  5
/** The DC offset of the ampified PIR sensor output */
#define PIR_AMP_OFFSET_PIN  4
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

#endif /* PLATFORM_SENSE_REV3_H_ */
/**
 * @}
 * @}
 */
