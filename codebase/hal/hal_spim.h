/**
 *  hal_spim.h : SPI Master HAL
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
 * @defgroup group_spim_driver SPI Master HAL
 * @brief Hardware abstraction layer of the SPI in Master mode. This driver
 *  is completely event driven and non-blocking, suitable for low power applications.
 *
 * @{
 */

#ifndef HAL_SPIM_H
#define HAL_SPIM_H

#include "nrf.h"
#include "nrf_util.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef HAL_SPIM_PERIPH_USED 
#define HAL_SPIM_PERIPH_USED 0
#endif

#ifndef SPIM_USED 
#define SPIM_USED HAL_SPIM_PERIPH_USED
#endif


/** Enum containing list of all the possible transmission frequencies */
typedef enum
{
    HAL_SPIM_FREQ_125K = SPIM_FREQUENCY_FREQUENCY_K125,
    HAL_SPIM_FREQ_250K = SPIM_FREQUENCY_FREQUENCY_K250,
    HAL_SPIM_FREQ_500K = SPIM_FREQUENCY_FREQUENCY_K500,
    HAL_SPIM_FREQ_1M = SPIM_FREQUENCY_FREQUENCY_M1,
    HAL_SPIM_FREQ_2M = SPIM_FREQUENCY_FREQUENCY_M2,
    HAL_SPIM_FREQ_4M = SPIM_FREQUENCY_FREQUENCY_M4,
    HAL_SPIM_FREQ_8M = SPIM_FREQUENCY_FREQUENCY_M8
    
}hal_spim_freq_t;

/** Enum containing list of all SPI Modes */
typedef enum
{
    /** CPol : Active High, CPha : Leading */
    HAL_SPIM_SPI_MODE0,
    /** CPol : Active High, CPha : Trailing */
    HAL_SPIM_SPI_MODE1,
    /** CPol : Active Low, CPha : Leading */
    HAL_SPIM_SPI_MODE2,
    /** CPol : Active Low, CPha : Trailing */
    HAL_SPIM_SPI_MODE3,
}hal_spim_spi_mode_t;

/** Enum containing options for Byte order */
typedef enum
{
    /** Byte Order : MSB First */
    HAL_SPIM_MSB_FIRST,
    /** Byte Order : LSB First */
    HAL_SPIM_LSB_FIRST,
}hal_spim_byte_order_t;

/** Enum to contain list of interrupts which can be enabled from application */
typedef enum
{
    /** To enable Tx Done event */
    HAL_SPIM_TX_DONE = SPIM_INTENSET_ENDTX_Msk,
    /** To enable Rx Done event */
    HAL_SPIM_RX_DONE = SPIM_INTENSET_ENDRX_Msk,
}hal_spim_intr_t;

/** Structure which is to be used to initiate SPIM module */
typedef struct 
{
    /** CS Bar Pin No */
    uint32_t csBar_pin;
    /** MISO Pin No */
    uint32_t miso_pin;
    /** MOSI Pin No */
    uint32_t mosi_pin;
    /** SCK Pin No */
    uint32_t sck_pin;
    /** Clock Frequency */
    hal_spim_freq_t freq;
    /** Select SPI mode */
    hal_spim_spi_mode_t spi_mode;
    /** Select Byte Order
     * @val HAL_SPIM_MSB_FIRST
     * @val HAL_SPIM_LSB_FIRST */
    hal_spim_byte_order_t byte_order;
    /** App IRQ priority */
    app_irq_priority_t irq_priority;
    /** Interrupts which are to be enabled @ref hal_spim_intr_t
     * @val HAL_SPIM_TX_DONE
     * @val HAL_SPIM_RX_DONE 
     * @Note If application requires both, use logical OR */
    uint32_t en_intr;
    /** Function which is to be called after TX_Done event */
    void (*tx_done_handler )(uint32_t bytes_last_tx);
    /** Function which is to be called after RX_Done event */
    void (*rx_done_handler )(uint32_t bytes_last_rx);
}hal_spim_init_t;

/**
 * @brief Function to Initiate the SPIM module
 * @param spim_init Settings which is to be used to initiate the SPIM module. 
 */
void hal_spim_init (hal_spim_init_t * spim_init);

/**
 * @brief Function to start communication
 * @param p_tx_data Pointer to the data which is to be sent
 * @param tx_len Number of bytes which are to be sent
 * @param p_rx_data Pointer to the location where received data is to be stored.
 * @param rx_len Number of received bytes which are to be stored
 */
void hal_spim_tx_rx (void * p_tx_data, uint32_t tx_len, void * p_rx_data, uint32_t rx_len);

/**
 * @brief Function to check if SIPM module is available or not
 * @return Status of hal_spim module
 * @retval true hal_spim module is not available
 * @retval false hal_spim module is available
 */
uint32_t hal_spim_is_busy ();

/**
 * @brief Function to de-initialize the SPIM module
 */
void hal_spim_deinit ();

#endif /* HAL_SPIM_H */
/**
 * @}
 * @}
 */
