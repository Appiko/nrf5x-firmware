/**
 *  hal_pwm.h : PWM HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CODEBASE_HAL_HAL_PWM_H_
#define CODEBASE_HAL_HAL_PWM_H_

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_hal_pwm HAL PWM
 * @brief Hardware abstraction layer of the PWM peripheral in the nRF52 SoCs.
 * @{
 */

#include "stdint.h"
#include "stdbool.h"
#include "nrf.h"

#ifdef NRF51
#error "nRF51 series SoCs don't have a PWM peripheral"
#endif

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef HAL_PWM_PERIPH_USED 
#define HAL_PWM_PERIPH_USED 0
#endif


/** Specify which TWIM peripheral is used for this HAL module */
#define PWM_USED           HAL_PWM_PERIPH_USED

/** The maximum number of pins supported by this module */
#define HAL_PWM_MAX_PIN_NUM     4

/** @brief Select the PWM frequency to operate at. */
typedef enum
{
    HAL_PWM_FREQ_16MHz  = PWM_PRESCALER_PRESCALER_DIV_1,  /// 16 MHz / 1 = 16 MHz.
    HAL_PWM_FREQ_8MHz   = PWM_PRESCALER_PRESCALER_DIV_2,  /// 16 MHz / 2 = 8 MHz.
    HAL_PWM_FREQ_4MHz   = PWM_PRESCALER_PRESCALER_DIV_4,  /// 16 MHz / 4 = 4 MHz.
    HAL_PWM_FREQ_2MHz   = PWM_PRESCALER_PRESCALER_DIV_8,  /// 16 MHz / 8 = 2 MHz.
    HAL_PWM_FREQ_1MHz   = PWM_PRESCALER_PRESCALER_DIV_16, /// 16 MHz / 16 = 1 MHz.
    HAL_PWM_FREQ_500kHz = PWM_PRESCALER_PRESCALER_DIV_32, /// 16 MHz / 32 = 500 kHz.
    HAL_PWM_FREQ_250kHz = PWM_PRESCALER_PRESCALER_DIV_64, /// 16 MHz / 64 = 250 kHz.
    HAL_PWM_FREQ_125kHz = PWM_PRESCALER_PRESCALER_DIV_128 /// 16 MHz / 128 = 125 kHz.
} hal_pwm_freq_t;

/** @brief Select the operating mode of the PWM wave counter */
typedef enum
{
    /// Up counter, reset to 0 on incrementing to CounterTop
    HAL_PWM_MODE_UP      = PWM_MODE_UPDOWN_Up,
    /// Up & down counter, decrement back to 0 on reaching CounterTop
    HAL_PWM_MODE_UP_DOWN = PWM_MODE_UPDOWN_UpAndDown,
} hal_pwm_mode_t;

/**
 * @brief Bit masks for the PWM shortcuts. Or the appropriate ones for the
 *  required shortcuts.
 */
typedef enum
{
    /// Short between SEQEND[0] event and STOP task.
    HAL_PWM_SHORT_SEQEND0_STOP_MASK        = PWM_SHORTS_SEQEND0_STOP_Msk,
    /// Short between SEQEND[1] event and STOP task.
    HAL_PWM_SHORT_SEQEND1_STOP_MASK        = PWM_SHORTS_SEQEND1_STOP_Msk,
    /// Short between LOOPSDONE event and SEQSTART[0] task.
    HAL_PWM_SHORT_LOOPSDONE_SEQSTART0_MASK = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk,
    /// Short between LOOPSDONE event and SEQSTART[1] task.
    HAL_PWM_SHORT_LOOPSDONE_SEQSTART1_MASK = PWM_SHORTS_LOOPSDONE_SEQSTART1_Msk,
    /// Short between LOOPSDONE event and STOP task.
    HAL_PWM_SHORT_LOOPSDONE_STOP_MASK      = PWM_SHORTS_LOOPSDONE_STOP_Msk
} hal_pwm_short_mask_t;

/** @brief PWM decoder's data load modes. This mode selects how the different channels' next
 * value is loaded from the data read from the RAM. */
typedef enum
{
    /// A 16-bit value is used in all four (0-3) PWM channels.
    HAL_PWM_LOAD_COMMON     = PWM_DECODER_LOAD_Common,
    /// 1st 16-bit value used in channels 0 and 1; 2nd one in channels 2 and 3.
    HAL_PWM_LOAD_GROUPED    = PWM_DECODER_LOAD_Grouped,
    /// Each channel has its own 16 bit value.
    HAL_PWM_LOAD_INDIVIDUAL = PWM_DECODER_LOAD_Individual,
    /// 1st three channels have their own 16 bit values, 4th one is the wave counter.
    HAL_PWM_LOAD_WAVE_FORM  = PWM_DECODER_LOAD_WaveForm
} hal_pwm_decoder_load_t;

/** @brief PWM decoder's next load trigger modes. This selects when the next value is
 *  loaded from RAM. */
typedef enum
{
    /// After the loaded value is repeated for the number of times in REFRESH register
    HAL_PWM_STEP_INTERNAL = PWM_DECODER_MODE_RefreshCount,
    /// When the Next Step task is set
    HAL_PWM_STEP_EXTERNAL = PWM_DECODER_MODE_NextStep
} hal_pwm_dec_trigger_t;

/** @brief Configuration for a sequence of PWM values. Note that the buffer pointed to
 *  in this structure needs to have a lifetime longer than the duration of the PWM.
 */
typedef struct
{
    /// @brief Pointer to an array containing the PWM duty cycle values.
    ///  This array present in the data RAM should preferably have a
    ///  program lifetime (global or local static variable).
    /// @note Depending on the decoder load mode the 16 bit data for the various
    ///  channels must be present one after the other
    /// @note The MSB of the 16 bit value determines the polarity of the PWM output
    uint16_t * seq_values;
    /// Number of 16-bit values in the buffer pointed by @p seq_values.
    uint16_t len;
    /// @brief Number of times a particular value should be played.
    ///  Only for @ref HAL_PWM_STEP_INTERNAL mode.
    uint32_t repeats;
    /// @brief Additional number of cycles that the last PWM value is to be played
    ///  after the end. Only for @ref HAL_PWM_STEP_INTERNAL mode.
    uint32_t end_delay;
} hal_pwm_sequence_config_t;

/**
 * @brief PWM interrupts.
 */
typedef enum
{
    /// Interrupt on STOPPED event.
    HAL_PWM_IRQ_STOPPED_MASK      = PWM_INTENSET_STOPPED_Msk,
    /// Interrupt on SEQSTARTED[0] event.
    HAL_PWM_IRQ_SEQSTARTED0_MASK  = PWM_INTENSET_SEQSTARTED0_Msk,
    /// Interrupt on SEQSTARTED[1] event.
    HAL_PWM_IRQ_SEQSTARTED1_MASK  = PWM_INTENSET_SEQSTARTED1_Msk,
    /// Interrupt on SEQEND[0] event.
    HAL_PWM_IRQ_SEQEND0_MASK      = PWM_INTENSET_SEQEND0_Msk,
    /// Interrupt on SEQEND[1] event.
    HAL_PWM_IRQ_SEQEND1_MASK      = PWM_INTENSET_SEQEND1_Msk,
    /// Interrupt on PWMPERIODEND event.
    HAL_PWM_IRQ_PWMPERIODEND_MASK = PWM_INTENSET_PWMPERIODEND_Msk,
    /// Interrupt on LOOPSDONE event.
    HAL_PWM_IRQ_LOOPSDONE_MASK    = PWM_INTENSET_LOOPSDONE_Msk
} hal_pwm_irq_mask_t;

/**
 * @brief Struct for initializing the hal pwm module
 */
typedef struct
{
    /// Pointer to array containing the pins number used by hal pwm
    uint32_t * pins;
    /// Pointer to array having the state of pins when PWM generation isn't on
    bool * pin_idle_state;
    /// Number of pins to be used by hal pwm. Maximum is @ref HAL_PWM_MAX_PIN_NUM
    uint32_t pin_num;
    /// Select the operating frequency of the hal pwm module
    hal_pwm_freq_t oper_freq;
    /// Select the operating mode (Up or Up&Down) of the module's counter
    hal_pwm_mode_t oper_mode;
    /// IRQ priority with which the @p irq_handler in @ref hal_pwm_start_t is called
    uint32_t irq_priority;
}hal_pwm_init_t;

/**
 * @brief Struct containing the configuration for starting the hal pwm module
 */
typedef struct
{
    /// @brief Maximum count of the counter. This and oper_freq decide the
    /// frequency of the resulting PWM waveform
    uint32_t countertop;
    /// The number of times the pattern of both seq_config must be repeated
    uint32_t loop;
    /// @brief Select the shortcuts that need to be enabled.
    /// OR the values in @ref hal_pwm_short_mask_t to enable them.
    uint32_t shorts_mask;
    /// @brief Select the interrupts that need to be enabled
    /// OR the values in @ref hal_pwm_irq_mask_t to enable them.
    uint32_t interrupt_masks;
    /// @brief Select the way in which the data pointed in seq_config is
    /// loaded into to the various channels
    hal_pwm_decoder_load_t decoder_load;
    /// @brief Select when the next data is loaded to the PWM
    hal_pwm_dec_trigger_t decoder_trigger;
    /// @brief The two configurations for the sequences. If @p loop is zero
    /// only the first sequence is played once. Otherwise both the sequences
    /// are played repeatedly based on the @p loop value.
    hal_pwm_sequence_config_t seq_config[2];
    /// @brief the handler called based on the interrupt generated with the
    /// argument providing the source of the interrupt
    void (*irq_handler)(hal_pwm_irq_mask_t irq_source);
}hal_pwm_start_t;

/**
 * @brief Initialize the HAL PWM module. Can be called again to change the
 *  initialization configuration.
 * @param init_config Pointer to the configuration for the initialization
 */
void hal_pwm_init(hal_pwm_init_t * init_config);

/**
 * @brief Start the PWM generation based on the configuration provided
 * @param start_config Pointer to the configuration for the PWM to start
 */
void hal_pwm_start(hal_pwm_start_t * start_config);

/**
 * @brief Stop the PWM generation.
 */
void hal_pwm_stop(void);

#endif /* CODEBASE_HAL_HAL_PWM_H_ */

/**
 * @}
 * @}
 */
