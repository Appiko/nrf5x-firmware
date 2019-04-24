/**
 *  bluey_1v1.h : Bluey 1V1 board definitions
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
 * @defgroup board_bluey_1v1 Bluey rev 1.1
 * @brief The defines for the Bluey board 1.1 by Electonut
 * @{
 */

#ifndef PLATFORM_BLUEY_1V1
#define PLATFORM_BLUEY_1V1

/** @anchor bluey_1v1-leds
 * @name LED definitions for Bluey 1.1
 * @note LED_1, LED_2 and LED_3 are defines of the RGB LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED        19
#define LED_GREEN      18
#define LED_BLUE       17

#define LED_1          17
#define LED_2          18
#define LED_3          19
/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE 0
/** @} */

/** @anchor bluey_1v1-buttons
 * @name Button definitions for Bluey 1.1
 * @{*/
#define BUTTON_1       16
/** The input level when the button is pressed */
#define BUTTONS_ACTIVE_STATE 0
/** The kind of internal resistors required for the button */
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
/** @} */

/** @anchor bluey_1v1-serial
 * @name Serial port definitions for Bluey 1.1
 * @{*/
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
/** Bool to say if the hardware flow control is required */
#define HWFC           false
/** @} */

/** @anchor bluey_1v1-twi
 * @name I2C pin defines for Bluey 1.1. The sensors present on the
 *  I2C are a humidity and temperature sensor (HDC1010), ambient light
 *  sensor (APDS9300) and accelerometer-gyroscope sensor (LSM6DS3)
 * @{*/
#define SDA_PIN   13
#define SCL_PIN   11
/** @} */

/** @anchor bluey_1v1-hdc1010
 * @name Humidity and temperature sensor HDC1010 defines for Bluey 1.1
 * @{*/
///Data ready, active low pin
#define HDC1010_DRDYN_PIN   12
#define HDC1010_ADRS        0x40
/** @} */

/** @anchor bluey_1v1-apds9300
 * @name Ambient light sensor APDS9300 defines for Bluey 1.1
 * @{*/
#define APDS9300_INT_PIN    14
#define APDS9300_ADRS       0x39
/** @} */

/** @anchor bluey_1v1-lsm6ds3
 * @name Accelerometer-gyroscope LSM6DS3 defines for Bluey 1.1
 * @{*/
#define LSM6DS3_INT1_PIN    15
#define LSM6DS3_INT2_PIN    20
#define LSM6DS3_ADRS        0x6A
/** @} */

/** @anchor bluey_1v1-microsd
 * @name MicroSD card holder defines for Bluey 1.1
 * @{*/
#define MICROSD_CS_PIN      22
#define MICROSD_DI_PIN      23
#define MICROSD_DO_PIN      24
#define MICROSD_CLK_PIN     25
/** @} */

///Bool define if the circuitry is present for the internal DC-DC of nRF52
#define DC_DC_CIRCUITRY     true

///Bool define if a NFC Antenna circuitry is present
#define NFC_CIRCUITRY       true

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

#endif /* PLATFORM_BLUEY_1V1 */

/** @} */
/** @} */
