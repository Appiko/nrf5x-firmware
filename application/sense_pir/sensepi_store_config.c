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

#include "nvm_logger.h"

#include "nrf_util.h"
#include "nrf_assert.h"
#include "common_util.h"

#include "log.h"

#define LOG_ID LOG_ID_SENSEPI_STORE_CONFIG

#define PAGES_USED PAGES_USED_SENSEPI_STORE_CONFIG

#define START_PAGE START_PAGE_SENSEPI_STORE_CONFIG


uint32_t g_log_id = LOG_ID;
sensepi_store_config_t g_sensepi_store_config = {};
sensepi_store_config_t * p_sensepi_store_config = &g_sensepi_store_config;

/**
 * @brief Function to get the next location where firmware will store latest 
 * configuration.
 * 
 * @return Free memory location address in uint32_t format. 
 * @retval 0xFFFFFFFF memory is full and firmware will now clear all the 
 * previously saved configurations. 
 */
//static uint32_t get_next_location (void);

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

//static uint32_t get_next_location (void)
//{
//    log_printf("%s\n",__func__);
//    uint32_t * p_mem_loc = (uint32_t *) LAST_APP_PAGE_ADDR;
//    while(p_mem_loc < (uint32_t *)LAST_CONFIG_END_ADDR)
//    {
//        if(*(p_mem_loc) == MEM_RESET_VALUE)
//        {
//            return (uint32_t)p_mem_loc;
//        }
//        p_mem_loc += CONFIG_SIZE_TO_POINTER;
//    }
//    return MEM_FULL;
//    
//}


void sensepi_store_config_init ()
{
    log_config_t config_log = 
    {
        .log_id = g_log_id,
        .entry_size = sizeof(sensepi_store_config_t),
        .no_of_pages = PAGES_USED,
        .start_page = START_PAGE,
    };
    g_log_id = nvm_logger_log_init (&config_log);
    
    log_printf ("SensPi config log id : %d\n", g_log_id);
}

bool sensepi_store_config_is_memory_empty (void)
{
    return nvm_logger_is_log_empty (g_log_id);
}

void sensepi_store_config_write (sensepi_store_config_t* latest_config)
{
    log_printf("%s\n",__func__);
    nvm_logger_feed_data (g_log_id, latest_config);

    nvm_logger_fetch_tail_data (g_log_id, p_sensepi_store_config, NVM_LOGGER_GET_LAST_CONFIG);
}

sensepi_store_config_t * sensepi_store_config_get_last_config ()
{
    nvm_logger_fetch_tail_data (g_log_id, p_sensepi_store_config, NVM_LOGGER_GET_LAST_CONFIG);
    log_printf("%s - config time : %d/%d/%d %d, %d  %d\n", __func__,
               p_sensepi_store_config->ble_config.current_date.dd,
               p_sensepi_store_config->ble_config.current_date.mm,
               p_sensepi_store_config->ble_config.current_date.yy,
               p_sensepi_store_config->ble_config.current_time,
               p_sensepi_store_config->fw_ver_int,
               FW_VER);
    return (sensepi_store_config_t*) p_sensepi_store_config;
}

static void clear_all_config (void)
{
    log_printf("%s\n",__func__);
    nvm_logger_empty_log (g_log_id);
}

void sensepi_store_config_check_fw_ver ()
{
    log_printf("%s\n",__func__);
    p_sensepi_store_config = &g_sensepi_store_config;
    nvm_logger_fetch_tail_data (g_log_id, p_sensepi_store_config, NVM_LOGGER_GET_LAST_CONFIG);
    log_printf("%s - config time : %d/%d/%d %d, %d  %d\n", __func__,
               p_sensepi_store_config->ble_config.current_date.dd,
               p_sensepi_store_config->ble_config.current_date.mm,
               p_sensepi_store_config->ble_config.current_date.yy,
               p_sensepi_store_config->ble_config.current_time,
               p_sensepi_store_config->fw_ver_int,
               FW_VER);
    uint32_t local_major_num;
    local_major_num = g_sensepi_store_config.fw_ver_int/10000;
    if(local_major_num != (FW_VER/10000))
    {
        log_printf("Update FW version\n");
        clear_all_config ();
        update_fw_ver ();
    }
}

static void update_fw_ver ()  // make it as hal_nvmc_write
{
    log_printf("%s\n",__func__);
    
    g_sensepi_store_config.fw_ver_int = FW_VER;
    
    nvm_logger_feed_data (g_log_id, p_sensepi_store_config);
    
}
