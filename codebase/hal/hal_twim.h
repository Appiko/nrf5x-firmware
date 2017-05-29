/*
 *  hal_twim.h
 *
 *  Created on: 19-May-2017
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
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_twim_driver TWI Master HAL
 * @brief Hardware abstraction layer of the Two Wire Interface in Master mode. This driver
 *  is completely event driven and non-blocking, suitable for low power applications.
 *
 * @warning This module needs the LFCLK to be on and running to be able to work
 * @{
 */

#ifndef CODEBASE_HAL_HAL_TWIM_H_
#define CODEBASE_HAL_HAL_TWIM_H_

#include "nrf.h"

#ifdef NRF51
#error TWIM peripheral is not present in the nRF51 SoC
#endif

/** Specify which TWIM peripheral is used for this HAL module */
#define TWIM_USED           0

/** @brief Defines for TWI master clock frequency.
 */
typedef enum
{
    HAL_TWI_FREQ_100K = TWI_FREQUENCY_FREQUENCY_K100, ///< 100 kbps
    HAL_TWI_FREQ_250K = TWI_FREQUENCY_FREQUENCY_K250, ///< 250 kbps
    HAL_TWI_FREQ_400K = TWI_FREQUENCY_FREQUENCY_K400  ///< 400 kbps
} hal_twim_freq_t;

/** @brief Defines for the types of transfers possible.
 */
typedef enum {
    TWIM_TX,    ///< Only a Tx transfer
    TWIM_RX,    ///< Only a Rx transfer
    TWIM_TX_RX  ///< A Tx transfer followed by Rx with a repeated start
} twim_transfer_t;

/** @brief Defines for the types of errors possible during TWI transactions.
 */
typedef enum {
    TWIM_ERR_NONE,       ///< No error for this transfer
    TWIM_ERR_ADRS_NACK,  ///< The slave device generated an error on the address bytes
    TWIM_ERR_DATA_NACK   ///< The slave device generated an error on the data bytes
} twim_err_t;

/** @brief Defines for the return values for the transfer calls.
 */
typedef enum {
    TWIM_STARTED,   ///< Transfer successfully started
    TWIM_BUSY,      ///< A transfer is already happening
    TWIM_UNINIT     ///< The TWIM peripheral is not initialized
} twim_ret_status;

#define TWIM_TX_DONE_MSK            (1<<TWIM_TX)
#define TWIM_RX_DONE_MSK            (1<<TWIM_RX)
#define TWIM_TX_RX_DONE_MSK         (1<<TWIM_TX_RX)

/** @brief Structure for the TWI master driver initialization
 */
typedef struct
{
    uint32_t            scl;                 ///< SCL pin number
    uint32_t            sda;                 ///< SDA pin number
    hal_twim_freq_t     frequency;           ///< TWI frequency
    uint32_t            irq_priority;        ///< Interrupt priority
    uint32_t            address;             ///< I2C device address
    /// This event handler is called whenever an unmasked transaction has successfully
    /// completed or whenever an error occurs.
    /// @note Errors cannot be masked, while the types of transfer completes can be masked
    void (*evt_handler)(twim_err_t evt, twim_transfer_t transfer);
    /// Event Mask for specifying which successful transfer type calls the handler
    uint32_t            evt_mask;

} hal_twim_init_config_t;

/**
 * @brief Function for initializing and enabling one of the TWIM peripheral
 * @param config Pointer to the initialization configuration parameters
 */
void hal_twim_init(hal_twim_init_config_t * config);

/**
 * @brief Function for uninitializing and disabling one of the TWIM peripheral
 */
void hal_twim_uninit(void);

/**
 * @brief Start a Tx only TWI transfer
 * @param tx_ptr Pointer to the data to be transferred
 * @param tx_len Length of the data to be transferred
 * @return Status of the transfer as per @ref twim_ret_status
 */
twim_ret_status hal_twim_tx(uint8_t * tx_ptr, uint32_t tx_len);

/**
 * @brief Start a Rx only TWI transfer
 * @param rx_ptr Pointer to the data to be received
 * @param rx_len Length of the data to be received
 * @return Status of the transfer as per @ref twim_ret_status
 */
twim_ret_status hal_twim_rx(uint8_t * rx_ptr, uint32_t rx_len);

/**
 * @brief Start a Tx TWI transfer followed by a Rx by repeated start
 * @param tx_ptr Pointer to the data to be transferred
 * @param tx_len Length of the data to be transferred
 * @param rx_ptr Pointer to the data to be received
 * @param rx_len Length of the data to be received
 * @return Status of the transfer as per @ref twim_ret_status
 */
twim_ret_status hal_twim_tx_rx(uint8_t * tx_ptr, uint32_t tx_len,
        uint8_t * rx_ptr, uint32_t rx_len);

/**
 * @brief Get the current specified address of the I2C slave
 * @return The I2C address currently initialized in the driver
 */
uint32_t hal_twim_get_current_adrs(void);

#endif /* CODEBASE_HAL_HAL_TWIM_H_ */
/**
 * @}
 * @}
 */
