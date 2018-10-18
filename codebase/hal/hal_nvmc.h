/* 
 * File:   hal_nvmc.h
 * Copyright (c) 2018 Appiko
 * Created on 11 October, 2018, 6:40 PM
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

