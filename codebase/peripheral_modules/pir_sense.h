/**
 *  pir_sense.h : PIR sensor driver
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

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#include "aux_clk.h"
#endif 
#ifndef SAADC_CHANNEL_USED_PIR_SENSE
#define SAADC_CHANNEL_USED_PIR_SENSE 0
#endif

#ifndef PPI_CHANNEL_USED_PIR_SENSE_1
#define PPI_CHANNEL_USED_PIR_SENSE_1 0
#endif

#ifndef PPI_CHANNEL_USED_PIR_SENSE_2
#define PPI_CHANNEL_USED_PIR_SENSE_2 AUX_CLK_PPI_CHANNEL_0
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
