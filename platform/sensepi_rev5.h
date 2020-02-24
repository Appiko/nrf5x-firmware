/**
 *  sensepi_rev5.h : SensePi Rev 5 board definitions
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
 * @defgroup board_sensepi_rev5 Appiko SensePi revision 4
 * @brief Board definitions for the 4th revision of the passive motion sensor by Appiko
 * @{
 */

#ifndef PLATFORM_SENSE_REV5_H_
#define PLATFORM_SENSE_REV5_H_

/** @anchor sensepi_rev5_leds
 * @name LED definitions for Sense rev4
 * @note LED_1 and LED_2 are defines of the LED
 *  for compatibility with existing examples
 * @{*/
#define LED_RED			27
#define LED_GREEN               29

#define LED_1			27
#define LED_2			29

/** The output level at which the LEDs shine */
#define LEDS_ACTIVE_STATE	1
/** The light sensing */
#define LIGHT_SENSE_EN		26
#define LIGHT_SENSE		28
/** @} */

/** @anchor sensepi_rev5_config_enable
 * @name Definition of configuration mode enable button pin
 * @{ 
*/
#define BUTTON_PIN		12
/** The logic level at which the Button will set value */
#define BUTTON_ACTIVE_STATE	0
/** @} */

/** @anchor sensepi_rev5_audio_jack
 * @name Definitions of the pin of the audio jack to trigger camera
 * @{*/
#define JACK_AUDIO_JACK_IO_PIN	17
#define JACK_FOCUS_PIN		16
#define JACK_TRIGGER_PIN	15
/** @} */

/// Battery voltage level sensing pin
#define BATT_VOLTAGE_SENSE	31

/** @anchor sensepi_rev5_serial
 * @name Serial port definitions for Sense rev4
 * @{*/
#define RX_PIN_NUMBER		19
#define TX_PIN_NUMBER		20
#define HWFC			false
#define RTS_PIN_NUMBER		23
#define CTS_PIN_NUMBER		24
/** @} */


/** @anchor sensepi_rev5_amp_pir
 * @name Pins for the amplified and filtered PIR signal
 * @{*/
/** Power pin for PIR circuitry  */
#define PIR_VDD                         7
/** The amplified and filtered output of the PIR sensor */
#define PIR_AMP_SIGNAL_PIN		4
/** The DC offset of the amplified PIR sensor output */
#define PIR_AMP_OFFSET_PIN		5
/** @} */

/** @anchor sensepi_rev5_i2c
 * @name Pins for the I2C communication
 * @{*/
/** I2C SDA Pin */
#define SDA_PIN     8
/** I2C SCL Pin */
#define SCL_PIN     9
/** @} */

/** @anchor sensepi_rev5_ext_rtc
 * @name Pins for the I2C communication
 * @{*/
/** Init Pin for External RTC */
#define EXT_RTC_INT 11
/** Clock signal for External RTC */
#define EXT_RTC_CLK 10
/** @} */

/** @anchor sensepi_rev5_mcp4012
 * @name Pins for driving MCP4012
 * @{*/
/** Chip Select Pin for MCP4012 */
#define MCP4012T_CS_PIN		2
/** U/D pin to set wiper value for MCP4012 */
#define MCP4012T_UD_PIN         3
/** SCK pin required for SPI communication */
#define SPI_SCK_PIN		6
/** @} */

/** @anchor sensepi_rev5_gpio
 * @name Pins for testing and debugging
 * @{*/
#define GPIO1_PIN		25
#define GPIO2_PIN		13
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
