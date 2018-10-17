/* 
 * File:   sensepi_store_config.h
 * Copyright (c) 2018 Appiko
 * Created on 4 October, 2018, 11:40 AM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */

/**
 * @addtogroup sense_appln
 * @{
 * 
 * @defgroup store_sensepi_config The support code to store sensepi_configs in flash
 * @brief Functions to store and manage sensepi_configs in non volatile memory. 
 * This will enable the firmware to reload the previously written config in case 
 * of power reset or system reset.
 * @{
 */

#ifndef SENSEPI_STORE_CONFIG_H
#define SENSEPI_STORE_CONFIG_H

#include "sensepi_ble.h"

/**Address of first memory location of last page available for application*/
#define SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR 0x23000


/**
 * @breif Function to check if memory where config is to be written is empty.
 * @return Memory status.
 * @retval 0 Memory is not empty
 * @retval 1 Memory is empty
 */
bool sensepi_store_config_is_memory_empty (void);

/**
 * @brief Function to write the sensepi_config_t at address location received 
 * from @ref get_next_location().
 * 
 * @note all the previously stored configurations will be erased if return value 
 * of @ref sensepi_store_config_get_next_location() is 0xFFFFFFFF 
 * @param latest_config pointer to sensepi_config_t which is to be stored in memory.
 */
void sensepi_store_config_write (sensepi_config_t * latest_config);

/**
 * @breif Function to get the last sensepi_config_t stored in flash. 
 * 
 * @Warning This function cannot differentiate between single stored config and \
 * empty flash page. So make sure that there is at least one configuration \
 * stored in memory. Use @ref sensepi_store_config_is_memory_empty() function for 
 * that
 * 
 * @return pointer to last sensepi_config_t stored in flash.
 */
sensepi_config_t * sensepi_store_config_get_last_config (void);

/**
 * @brief Function to check the major number of firmware if latest major number \
 * firmware version is greater than respective previous number then it'll \
 * initiate reset for stored configs.
 * 
 */
void sensepi_store_config_check_fw_ver ();
#endif /* SENSEPI_STORE_CONFIG_H */
/**
 * @}
 * @}
 */
