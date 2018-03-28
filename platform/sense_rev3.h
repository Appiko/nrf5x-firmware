/*
 *  sense_rev3.h
 *
 *  Created on: 29-Jan-2018
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
#define RX_PIN_NUMBER  17
#define TX_PIN_NUMBER  19
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
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#define BOARD_LFCLKSRC         NRF_CLOCK_LFCLK_Xtal

#endif /* PLATFORM_SENSE_REV3_H_ */
/**
 * @}
 * @}
 */
