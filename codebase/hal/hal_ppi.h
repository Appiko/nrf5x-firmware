/*
 *  hal_ppi.h : Basic HAL driver to provide abstraction for PPI peripheral
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

#ifndef HAL_PPI_H
#define HAL_PPI_H

#include "stdint.h"

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_hal_ppi HAL PPI
 * @brief Hardware abstraction layer of the PPI peripheral in the nRF52 SoCs.
 * @{
 */

/** List of possible errors */
typedef enum
{
    /** No Error */
    PPI_SETUP_SUCCESSFUL = 0x00,
    /** Invalid Event */
    PPI_INVALID_EVENT = 0x01,
    /** Invalid Task */
    PPI_INVALID_TASK = 0x02, 
    /** Invalid Fork */
    PPI_INVALID_FORK = 0x04,        
    /** Invalid Channel */
    PPI_INVALID_CH = 0x08,  
    /** Invalid Peripheral */
    PPI_INVALID_PERIPHERAL = 0x10,  
            
}ppi_setup_status_t;

/** Structure containing information needed to setup certain PPI channel */
typedef struct
{
    /** PPI channel number */
    uint32_t ppi_id;
    /** Event after which tasks are to be initiated automatically */
    uint32_t event;
    /** Primary Task which is to be initiated automatically */
    uint32_t task;
    /** Secondary task which is to be initiated automatically */
    uint32_t fork;
}hal_ppi_setup_t;

/**
 * @brief Function to setup a PPI
 * @param setup Structure pointer of data type @ref hal_ppi_setup_t
 * @return Status of PPI setup @ref ppi_setup_status_t
 */
ppi_setup_status_t hal_ppi_set (hal_ppi_setup_t * setup);

/**
 * @brief Function to Enable given PPI channel
 * @param ppi_id PPI channel which is to be enabled
 */
void hal_ppi_en_ch (uint32_t ppi_id);

/**
 * @brief Function to disable given PPI channel
 * @param ppi_id PPI channel which is to be disabled
 */
void hal_ppi_dis_ch (uint32_t ppi_id);

/**
 * @brief Function to set triggering event for given PPI channel 
 * @param ppi_id PPI channel for which triggering event is to be set
 * @param new_event New triggering event
 */
void hal_ppi_set_event (uint32_t ppi_id, uint32_t new_event);

/**
 * @brief Function to change primary task for given PPI channel
 * @param ppi_id PPI channel for which primary task is to be changed
 * @param new_task New primary task
 */
void hal_ppi_set_task (uint32_t ppi_id, uint32_t new_task);

/**
 * @brief Function to set secondary task for given PPI channel
 * @param ppi_id PPI channel for which secondary task is to be set
 * @param new_fork New secondary task
 */
void hal_ppi_set_fork (uint32_t ppi_id, uint32_t new_fork);

/**
 * @}
 * @}
 */

#endif /* HAL_PPI_H */
