/*  Copyright (c) 2017, Appiko
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
#define TX_PIN_NUMBER  29
#define CTS_PIN_NUMBER 30
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
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define BOARD_LFCLKSRC         NRF_CLOCK_LFCLK_Xtal

#endif /* PLATFORM_BLUEY_1V1 */

/** @} */
/** @} */
