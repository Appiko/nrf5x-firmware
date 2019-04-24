/*
 *  sensepi_store_config.c : Support file to store configs into NVMC memory 
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


#include "sensepi_store_config.h"

#include "hal_nop_delay.h"
#include "hal_nvmc.h"

#include "nrf_util.h"
#include "nrf_assert.h"
#include "common_util.h"

#include "log.h"

/**Address of first memory location of last page available for application*/
#define LAST_APP_PAGE_ADDR 0x27000
/**Size of config in unit of size of pointer*/
#define CONFIG_SIZE_TO_POINTER 5 
/**Number of configurations to be stored*/
#define NO_OF_CONFIGS 204
/**Word size*/
#define WORD_SIZE 4
/** End of memory location to store the configurations*/
#define LAST_CONFIG_END_ADDR (LAST_APP_PAGE_ADDR+NO_OF_CONFIGS*CONFIG_SIZE_TO_POINTER\
                                * WORD_SIZE)  ///After storing configuration on this location 
///next time before writing, all the configurations will be erased.
/**Address where local firmware version number is saved*/
#define CONFIG_FW_VER_LOC LAST_CONFIG_END_ADDR+0x2
/** Reset value or any flash register */
#define MEM_RESET_VALUE 0xFFFFFFFF
/** Memory full flag*/
#define MEM_FULL LAST_CONFIG_END_ADDR

/**
 * @brief Function to get the next location where firmware will store latest 
 * configuration.
 * 
 * @return Free memory location address in uint32_t format. 
 * @retval 0xFFFFFFFF memory is full and firmware will now clear all the 
 * previously saved configurations. 
 */
static uint32_t get_next_location (void);

/**
 * @brief Function to erase all the previously return configurations.
 * 
 * @note This function will get called automatically once memory is full.
 */
static void clear_all_config (void);

/**
 * @brief Function to update the firmware version stored in this page.
 */
static void update_fw_ver (void);

static uint32_t get_next_location (void)
{
    log_printf("%s\n",__func__);
    uint32_t * p_mem_loc = (uint32_t *) LAST_APP_PAGE_ADDR;
    while(p_mem_loc < (uint32_t *)LAST_CONFIG_END_ADDR)
    {
        if(*(p_mem_loc) == MEM_RESET_VALUE)
        {
            return (uint32_t)p_mem_loc;
        }
        p_mem_loc += CONFIG_SIZE_TO_POINTER;
    }
    return MEM_FULL;
    
}

bool sensepi_store_config_is_memory_empty (void)
{
    log_printf("%s\n",__func__);
    if(get_next_location () == LAST_APP_PAGE_ADDR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void sensepi_store_config_write (sensepi_config_t* latest_config)
{
    log_printf("%s\n",__func__);
    uint32_t * p_mem_loc = (uint32_t *) get_next_location();
    if(p_mem_loc == (uint32_t *)MEM_FULL)
    {
        clear_all_config ();
        p_mem_loc = (uint32_t *) get_next_location();
    }
    hal_nvmc_write_data(p_mem_loc, latest_config, sizeof(sensepi_config_t));

}

sensepi_config_t * sensepi_store_config_get_last_config ()
{
    log_printf("%s\n",__func__);
    uint32_t * p_mem_loc = (uint32_t*)get_next_location();
    if(p_mem_loc != (uint32_t*)LAST_APP_PAGE_ADDR)
    {
        p_mem_loc -= CONFIG_SIZE_TO_POINTER;
    }
    return (sensepi_config_t*) p_mem_loc;
}

static void clear_all_config (void)
{
    log_printf("%s\n",__func__);
    hal_nvmc_erase_page (LAST_APP_PAGE_ADDR);
}

void sensepi_store_config_check_fw_ver ()
{
    log_printf("%s\n",__func__);
    uint32_t * p_mem_loc = (uint32_t *) CONFIG_FW_VER_LOC;
    uint32_t local_major_num = *p_mem_loc/10000;
    if(*p_mem_loc == MEM_RESET_VALUE)
    {
        update_fw_ver ();
    }
    else if(local_major_num != (FW_VER/10000))
    {
        clear_all_config ();
        update_fw_ver ();
    }
}

static void update_fw_ver ()  // make it as hal_nvmc_write
{
    log_printf("%s\n",__func__);
    uint32_t * p_mem_loc = (uint32_t *) CONFIG_FW_VER_LOC;
    uint32_t local_fw_ver = FW_VER;
    hal_nvmc_write_data (p_mem_loc, &local_fw_ver, sizeof(uint32_t));
    
}
