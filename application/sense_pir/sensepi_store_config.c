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

#include "nrf_util.h"
#include "nrf_assert.h"
#include "common_util.h"

#include "log.h"

/**Starting of memory location to store the last configuration*/
#define LAST_CONFIG_ADDR 0x27FE8  ///After storing configuration on this location 
///next time before writing, all the configurations will be erased.
/** Reset value or any flash register */
#define MEM_RESET_VALUE 0xFFFFFFFF
/**Size of config in unit of size of pointer*/
#define CONFIG_SIZE_TO_POINTER 5 

/**Pointer to do all flash memory related operations*/
static uint32_t * p_mem_loc;

uint32_t sensepi_store_config_get_next_location (void)
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t *) SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR;
    while(p_mem_loc <= (uint32_t *)LAST_CONFIG_ADDR)
    {
        if(*p_mem_loc == MEM_RESET_VALUE)
        {
            return (uint32_t)p_mem_loc;
        }
        p_mem_loc += CONFIG_SIZE_TO_POINTER;
        hal_nop_delay_us(700);
    }
    return MEM_RESET_VALUE;
    
}

void sensepi_store_config_write (sensepi_config_t* latest_config)
{
    log_printf("%s\n",__func__);
    p_mem_loc = (uint32_t *) sensepi_store_config_get_next_location();
    uint32_t * p_config_cast = (uint32_t *) latest_config;
    if(p_mem_loc == (uint32_t *)MEM_RESET_VALUE)
    {
        sensepi_store_config_clear_all ();
        p_mem_loc = (uint32_t *) sensepi_store_config_get_next_location();
    }
    
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    for(uint32_t i =0 ; i < CONFIG_SIZE_TO_POINTER; i++)
    {
        *p_mem_loc++ = *p_config_cast++;
    }
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);

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

void sensepi_store_config_clear_all (void)
{
    log_printf("%s\n",__func__);
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
    NRF_NVMC->ERASEPAGE = (uint32_t)SENSEPI_STORE_CONFIG_LAST_APP_PAGE_ADDR;
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
}
