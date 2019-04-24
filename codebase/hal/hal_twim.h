/**
 *  hal_twim.h : TWI Master HAL
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

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_twim_driver TWI Master HAL
 * @brief Hardware abstraction layer of the Two Wire Interface in Master mode. This driver
 *  is completely event driven and non-blocking, suitable for low power applications.
 *
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

/** @anchor twim_evt_mask
 * @name Masks for specifying which events calls the handler. These need to be
 *  ORed to enable multiple events' handlers to be called.
 * @{*/
#define TWIM_TX_DONE_MSK      (1<<TWIM_TX)
#define TWIM_RX_DONE_MSK      (1<<TWIM_RX)
#define TWIM_TX_RX_DONE_MSK   (1<<TWIM_TX_RX)
/** @} */

/** @brief Structure for the TWI master driver initialization
 */
typedef struct
{
    uint32_t            scl;                 ///< SCL pin number
    uint32_t            sda;                 ///< SDA pin number
    hal_twim_freq_t     frequency;           ///< TWI frequency
    uint32_t            irq_priority;        ///< Interrupt priority
    uint32_t            address;             ///< I2C device address
    /// This event handler is called whenever an masked transaction has successfully
    /// completed or whenever an error occurs.
    /// @note Errors cannot be masked, while the types of transfer completes can be masked
    void (*evt_handler)(twim_err_t evt, twim_transfer_t transfer);
    /// Event Mask for specifying which successful transfer type calls the handler
    uint32_t            evt_mask;
} hal_twim_init_config_t;

/**
 * @brief Function for initializing and enabling one of the TWIM peripheral
 * @param config Pointer to the initialization configuration parameters
 * @warning The evt_handler in hal_twim_init_config_t should be initialized to
 *  a function pointer and must not be NULL.
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
