/*
 *  pir_sense.h
 *
 *  Created on: 31-Jan-2018
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
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_pir_sense PIR Sense
 *
 * @brief Driver use detect motion based on threshold from a PIR sensor based
 *  on PPI based automated ADC sampling of the PIR signal from the triggering
 *  of a RTC timer.
 *
 *  The image below gives the flow of how the peripherals of nRF52 is setup for
 *  this application.
 *
 *
 * @dot
 * digraph State_machine_diagram {
 *  rankdir="LR";
 *  rtc_trg [shape = point]
 *  adc_trg [shape = point]
 *  rtc_evt [shape = circle, width = 1.5, label = "RTC\ntimeout"]
 *  rtc_clr [shape = circle, width = 1.5, label ="Clear RTC"]
 *  adc_start [shape = circle, width = 1.5, label ="Start ADC"]
 *  adc_sample [shape = circle, width = 1.5, label ="Sample ADC"]
 *  adc_end [shape = circle, width = 1.5, label = "ADC Ended"]
 *  adc_isr [shape = circle, width = 1.5, label = "ADC IRQ"]
 *  adc_stop [shape = circle, width = 1.5, label ="Stop ADC"]
 *  rtc_trg -> rtc_evt [style = "dotted", label = "Event on RTC\ncompare every\nsampling interval"]
 *  rtc_evt -> rtc_clr [label = "PPI:start counting\nagain from 0"];
 *  rtc_evt -> adc_start [label = "PPI fork:\nready the ADC"];
 *  adc_start -> adc_sample [label = "PPI:sample\nwith ADC"];
 *  adc_sample -> adc_end [style = "dotted", label="Event on\nsampling"];
 *  adc_sample -> adc_isr [style = "invisible", dir = "none"];
 *  adc_trg -> adc_isr [style = "dotted", label = "ISR:when ADC high/low\nlimits are crossed"]
 *  adc_end -> adc_stop [label = "PPI:stop ADC\non sampling"];
 * }
 * @enddot
 *
 * @warning This module needs the LFCLK to be on and running to be able to work
 *
 * @warning This module uses RTC0, which is used by Softdevice. So this module
 *  will not work with a Softdevice.
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_PIR_SENSE_H_
#define CODEBASE_PERIPHERAL_MODULES_PIR_SENSE_H_

#include "stdint.h"
#include "stdbool.h"

#if MAIN_H_PRESENT == 1
#include "main.h"
#endif 
#ifndef SAADC_CHANNEL_USED_PIR_SENSE
#define SAADC_CHANNEL_USED_PIR_SENSE 0
#endif

#ifndef PPI_CHANNEL_USED_PIR_SENSE_1
#define PPI_CHANNEL_USED_PIR_SENSE_1 0
#endif

#ifndef PPI_CHANNEL_USED_PIR_SENSE_2
#define PPI_CHANNEL_USED_PIR_SENSE_2 1
#endif

#ifndef PPI_CHANNEL_USED_PIR_SENSE_3
#define PPI_CHANNEL_USED_PIR_SENSE_3 2
#endif

#ifndef RTC_USED_PIR_SENSE
#define RTC_USED_PIR_SENSE 0
#endif

/**
 * @brief Stucture for passing the configuration for initializing the
 *  PIR Sense module.
 */
typedef struct {
  uint32_t sense_interval_ms;   /// The sampling interval in ms
  uint32_t pir_signal_analog_in;   ///The analog input number of PIR signal
  uint32_t pir_offset_analog_in;   ///The analog input number of PIR offset
  uint32_t threshold;           ///The (+/-) threshold to be crossed
                                ///for the handler to be called
  uint32_t irq_priority;        ///The interrupt priority for calling the handler
  void (*handler)(int32_t adc_val); ///The pointer of the handler function to be
                                ///called when motion is detected
}pir_sense_cfg;

/**
 *  Initialize and start the PIR Sense module based on the configuration received
 * @param init Initialization configuration pointer
 */
void pir_sense_start(pir_sense_cfg * init);


/**
 * Deinitializes, stops the PIR Sense module and frees the peripherals used by it
 */
void pir_sense_stop(void);

#endif /* CODEBASE_PERIPHERAL_MODULES_PIR_SENSE_H_ */
/**
 * @}
 * @}
 */
