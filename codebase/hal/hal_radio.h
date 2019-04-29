/*
 *  hal_radio.h : Basic driver for Radio peripheral
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
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_radio HAL Radio
 * @brief Hardware abstraction layer of Radio peripheral.
 * @{
 */

#ifndef HAL_RADIO_H
#define HAL_RADIO_H

#include "stdbool.h"
#include "nrf_util.h"
#include "stdint.h"


/**
 * @brief Structure used to store the data required for radio configuration
 */
typedef struct
{
    /** Freq value which is to be added in 2400MHz */
    uint32_t freq;
    /** IRQ Priority level */
    app_irq_priority_t irq_priority;
    /** Pointer to the function which is to be called once transmission is done */
    void (* tx_done_handler) (void * p_buff, uint32_t len);
    /** Pointer to the function which is to be called once reception is done */
    void (* rx_done_handler) (void * p_buff, uint32_t len);
}hal_radio_config_t;

/**
 * @brief Function to Initiate Radio peripheral
 * @param radio_init_config Configuration used to initiate the radio peripheral
 */
void hal_radio_init (hal_radio_config_t * radio_init_config);

/**
 * @brief Function to Set the payload data for transmission
 * @param p_payload Pointer to sequential data which is to be sent.
 * @param len Length of data in bytes
 */
void hal_radio_set_payload_data (void * p_payload, uint32_t len);

/**
 * @brief Function to start data transmission
 */
void hal_radio_start_tx ();

/**
 * @brief Function to start data reception
 */
void hal_radio_start_rx ();

/**
 * @brief Function to stop radio peripheral
 */
void hal_radio_stop ();


//For future development

/***/
//void hal_radio_update_freq (uint32_t new_freq);

#endif /* HAL_RADIO_H */


/**
 * @}
 * @}
 */
