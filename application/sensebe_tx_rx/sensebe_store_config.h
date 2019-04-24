/*
 *  sensebe_store_config.h : Support file to store configs into NVMC memory 
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
 * @addtogroup sensebe_appln
 * @{
 *
 * @defgroup store_config The support code to store configurations.
 * @brief The support code to store configurations received from other BLE device
 * @{
 *
 */


#ifndef SENSEBE_STORE_CONFIG_H
#define SENSEBE_STORE_CONFIG_H
#include "sensebe_ble.h"
/**
 * @breif Function to check if memory where config is to be written is empty.
 * @return Memory status.
 * @retval 0 Memory is not empty
 * @retval 1 Memory is empty
 */
bool sensebe_store_config_is_memory_empty (void);

/**
 * @brief Function to write the sensebe_config_t at address location received 
 * from @ref get_next_location().
 * 
 * @note all the previously stored configurations will be erased if return value 
 * of @ref sensebe_store_config_get_next_location() is 0xFFFFFFFF 
 * @param latest_config pointer to sensebe_config_t which is to be stored in memory.
 */
void sensebe_store_config_write (sensebe_config_t * latest_config);

/**
 * @breif Function to get the last sensebe_config_t stored in flash. 
 * 
 * @Warning This function cannot differentiate between single stored config and \
 * empty flash page. So make sure that there is at least one configuration \
 * stored in memory. Use @ref sensebe_store_config_is_memory_empty() function for 
 * that
 * 
 * @return pointer to last sensebe_config_t stored in flash.
 */
sensebe_config_t * sensebe_store_config_get_last_config (void);

/**
 * @brief Function to check the major number of firmware if latest major number \
 * firmware version is greater than respective previous number then it'll \
 * initiate reset for stored configs.
 * 
 */
void sensebe_store_config_check_fw_ver ();


#endif /* SENSEBE_STORE_CONFIG_H */
/**
 * @}
 * @}
 */

