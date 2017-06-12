/*
 *  detect_rev1.h
 *
 *  Created on: 26-Mar-2017
 *
 *  Copyright (c) 2017, Appiko
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
 * @defgroup board_detect_rev1 Appiko Detect Revision 1
 * @brief The first revision of the animal detector unit by Appiko
 * @{
 */

#ifndef PLATFORM_DETECT_REV1_H_
#define PLATFORM_DETECT_REV1_H_

/** @anchor detect_rev1_leds
 * @name LED definitions for Detect rev1.
 * The connection is: Anode pin -> 100E resistor -> LED -> Cathode pin.
 * The LED anode pin is the Sense pin.
 * @{*/
#define LED_GREEN_ANODE   12
#define LED_GREEN_CATHODE 14
#define LED_GREEN_SENSE   3

#define LED_ORANGE_ANODE   11
#define LED_ORANGE_CATHODE 13
#define LED_ORANGE_SENSE   2
/** @} */

/** @anchor detect_rev1_buttons
 * @name Button definitions for Detect rev1
 * @{*/
#define BUTTON_PIN     17
/** The input level when the button is pressed */
#define BUTTONS_ACTIVE_STATE 0
/** The kind of internal resistors required for the button */
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
/** @} */

/** Pin for debugging purpose */
#define DEBUG_PIN      18

/** @anchor detect_rev1_serial
 * @name Serial port definitions for Detect rev1
 * @{*/
#define RX_PIN_NUMBER  19
#define TX_PIN_NUMBER  22
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 5
/** Bool to say if the hardware flow control is required */
#define HWFC           false
/** @} */

/** @anchor detect_rev1_wide_pir
 * @name Wide angle PIR definitions for Detect rev1.
 * @{*/
/** This pin provides the supply for the PIR sensor and RC filter*/
#define WIDE_VDD_PIN     6
/** The RC filtered output of the PIR sensor */
#define WIDE_SIGNAL_PIN  4
/** The DC offset of the PIR sensor output */
#define WIDE_OFFSET_PIN  5
/** @} */

/** @anchor detect_rev1_narrow_pir
 * @name Narrow angle PIR definitions for Detect rev1.
 * @{*/
/** This pin provides the supply for the PIR sensor and RC filter*/
#define NARROW_VDD_PIN     27
/** The RC filtered output of the PIR sensor */
#define NARROW_SIGNAL_PIN  28
/** The DC offset of the PIR sensor output */
#define NARROW_OFFSET_PIN  29
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

#endif /* PLATFORM_DETECT_REV1_H_ */

/**
 * @}
 * @}
 */
