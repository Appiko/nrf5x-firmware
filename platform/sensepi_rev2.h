/**
 *  sensepi_rev2.h : SensePi Rev 2 board definitions
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
 * @defgroup board_sense_rev2 Appiko Sense Revision 2
 * @brief The second revision of the animal detector unit by Appiko
 * @{
 */

#ifndef PLATFORM_SENSE_REV2_H_
#define PLATFORM_SENSE_REV2_H_

/** @anchor sense_rev2_leds
 * @name LED definitions for Sense rev2 which uses HackaBLE
 * @note LED_1, LED_2 and LED_3 are defines of the RGB LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED        19
#define LED_GREEN      20
#define LED_BLUE       17

#define LED_1          17
#define LED_2          20
#define LED_3          19
/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE 0
/** @} */

/** @anchor sense_rev2_serial
 * @name Serial port definitions for Sense rev2
 * @{*/
#define RX_PIN_NUMBER  26
#define TX_PIN_NUMBER  27
#define CTS_PIN_NUMBER 30
#define RTS_PIN_NUMBER 29
#define HWFC false
/** @} */

/** @anchor sense_rev2_rc_pir
 * @name Pins for the signal directly from the PIR through a RC filter
 * @{*/
/** The RC filtered output of the PIR sensor */
#define PIR_RC_SIGNAL_PIN  4
/** The DC offset of the PIR sensor output */
#define PIR_RC_OFFSET_PIN  5
/** @} */

/** @anchor sense_rev2_amp_pir
 * @name Pins for the amplified and filtered PIR signal
 * @{*/
/** The amplified and filtered output of the PIR sensor */
#define PIR_AMP_SIGNAL_PIN  3
/** The DC offset of the ampified PIR sensor output */
#define PIR_AMP_OFFSET_PIN  2
/** @} */

///Controls the high/low power mode of the TPS610985 boost converter
/// It is pulled low (low power mode) in the circuitry
#define TPS610985_REG_MODE_PIN 25

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
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define BOARD_LFCLKSRC         NRF_CLOCK_LFCLK_Xtal

#endif /* PLATFORM_SENSE_REV2_H_ */
/**
 * @}
 * @}
 */
