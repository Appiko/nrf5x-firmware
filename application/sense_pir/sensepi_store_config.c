/* 
 * File:   sensepi_store_config.c
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


#include "sensepi_store_config.h"

#include "hal_nop_delay.h"
#include "hal_nvmc.h"

#include "nrf_util.h"
#include "nrf_assert.h"
#include "common_util.h"

#include "log.h"

/**Starting of memory location to store the last configuration*/
#define LAST_CONFIG_ADDR (SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR+0xFF0)  ///After storing configuration on this location 
///next time before writing, all the configurations will be erased.
/**Address where local firmware version number is saved*/
#define CONFIG_FW_VER_LOC LAST_CONFIG_ADDR+0x4
/** Reset value or any flash register */
#define MEM_RESET_VALUE 0xFFFFFFFF
/**Size of config in unit of size of pointer*/
#define CONFIG_SIZE_TO_POINTER 5 
/** Memory full flag*/
#define MEM_FULL 0xFFFFFFFF

/**Pointer to do all flash memory related operations*/
static uint32_t * p_mem_loc;

uint32_t sensepi_store_config_get_next_location (void)
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t *) SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR;
    while(p_mem_loc <= (uint32_t *)LAST_CONFIG_ADDR)
    {
        if(*(p_mem_loc) == MEM_FULL)
        {
            return (uint32_t)p_mem_loc;
        }
        p_mem_loc += CONFIG_SIZE_TO_POINTER;
        hal_nop_delay_us(700);  //check without delay
    }
    return MEM_FULL;
    
}

void sensepi_store_config_write (sensepi_config_t* latest_config)
{
    log_printf("%s\n",__func__);
    p_mem_loc = (void *) sensepi_store_config_get_next_location();
    hal_nvmc_write_data(p_mem_loc, latest_config, sizeof(sensepi_config_t));

}

sensepi_config_t * sensepi_store_config_get_last_config ()
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t*)sensepi_store_config_get_next_location();
    if(p_mem_loc != (uint32_t*)SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR)
    {
        p_mem_loc -= CONFIG_SIZE_TO_POINTER;
    }
    return (sensepi_config_t*) p_mem_loc;
}

void sensepi_store_config_clear_all (void) //make it as hal_nvmc_page_erase
{
    log_printf("%s\n",__func__);
    hal_nvcm_erase_page (SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR);
}

void sensepi_store_config_check_fw_ver ()
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t *) CONFIG_FW_VER_LOC;
    uint32_t local_major_num = *p_mem_loc/10000;
    if(*p_mem_loc == MEM_RESET_VALUE)
    {
        sensepi_store_config_update_fw_ver ();
    }
    else if(local_major_num != (FW_VER/10000))
    {
        sensepi_store_config_clear_all ();
        sensepi_store_config_update_fw_ver ();
    }
}

void sensepi_store_config_update_fw_ver ()  // make it as hal_nvmc_write
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t *) CONFIG_FW_VER_LOC;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
    *p_mem_loc = (uint32_t)FW_VER;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
}
