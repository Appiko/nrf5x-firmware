/**
 *  hackable.h : hackable board definitions
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
#define RX_PIN_NUMBER		2
#define TX_PIN_NUMBER		3
#define HWFC			false
#define RTS_PIN_NUMBER		6
#define CTS_PIN_NUMBER		7
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
