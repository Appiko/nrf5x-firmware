/**
 *  hal_nvmc.h : NVMC HAL
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


#ifndef HAL_NVMC_H
#define HAL_NVMC_H

#include "nrf.h"

#define HAL_NVMC_MEM_RESET_VAL 0xFFFFFFFF

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function to Erase the page starting with certain address
 * @param page_start_address
 * @return Status of operation.
 * @retval 0 for Success
 * @retval Other for failure
 */
uint32_t hal_nvmc_erase_page (uint32_t page_start_address);

/**
 * @brief Function to write data to flash.
 * @note Please be sure that some other memory isn't getting overwritten.
 * @param p_destination Pointer to starting address of destination memory.
 * @param p_source Pointer to starting address of source memory.
 * @param size_of_data Size of data which is to be calculated
 */
void hal_nvmc_write_data (void * p_destination, void * p_source, uint32_t size_of_data);
#ifdef __cplusplus
}
#endif

#endif /* HAL_NVMC_H */

