/**
 *  nvm_logger.h : NVM Logger module
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

#ifndef NVM_LOGGER_H
#define NVM_LOGGER_H

#include "stdint.h"
#include "stdbool.h"


#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

#ifndef NVM_LOGGER_PAGE_OFFSETS 
#define NVM_LOGGER_PAGE_OFFSETS 0x1000
#endif

#ifndef NVM_LOGGER_LAST_NVM_PAGE 
#define NVM_LOGGER_LAST_NVM_PAGE 0x27000
#endif

#ifndef NVM_LOGGER_MAX_PAGES
#define NVM_LOGGER_MAX_PAGES 6
#endif

#ifndef NVM_LOGGER_MAX_LOGS
#define NVM_LOGGER_MAX_LOGS 4
#endif

#ifndef NVM_LOGGER_PAGE_METADATA_ADDR
#define NVM_LOGGER_PAGE_METADATA_ADDR 0xFF0
#endif

#define NVM_LOGGER_DIR_FIRST_TO_LAST true
#define NVM_LOGGER_DIR_LAST_TO_FIRST false

#define NVM_LOGGER_GET_LAST_CONFIG 1
#define NVM_LOGGER_GET_ALL_CONFIG 0


typedef enum
{
    NVM_LOG_PAGE0 = 0x27000,
    NVM_LOG_PAGE1 = 0x26000,
    NVM_LOG_PAGE2 = 0x25000,
    NVM_LOG_PAGE3 = 0x24000,
    NVM_LOG_PAGE4 = 0x23000,
    NVM_LOG_PAGE5 = 0x23000,
    NVM_LOG_MAX_PAGES = 6,
}log_page_start_t;

typedef struct
{
    uint32_t log_id;
    uint32_t entry_size;
    uint32_t no_of_pages;
    uint32_t start_page;
}log_config_t;

/**
 * @brief Function to initiate the nvm_logger module. This func is to be called\
 * on system_reset to update the metadata of all the logs.
 */
void nvm_logger_mod_init (void);

/**
 * @brief Function to setup the log.
 * @param log_id Unique Log ID which is to be used to keep track of specific data type.
 * @Note @ref log_id has to be less than NVM_LOGGER_MAX_LOGS. If log_id is not\
 * specified then module will assign the log_id automatically.
 * @param entry_size Size of data_type which is to be stored. Default value is 32 
 * @param log_size Maximum number of data entries that user wants to keep. Default value is 254
 * @Note If @ref log_size is not specified but @ref data_size is specified other\
 * than 32 then logger store maximum number of logs possible in one page only i.e\
 * 4080/data_size logs
 * @return log_id Log ID which is supposed to used for the log which is being setup. 
 * @retval updated_log_id If log_id is not specified or having conflicts with previously\
 * declared logs, module will update the log_id.
 * @retval original_log_id If log_id is available then it'll not get updated.
 */
uint32_t nvm_logger_log_init (log_config_t * log_config);

/**
 * @brief Function to make data entry
 * @param log_id Log ID of log where you want to do this entry.
 * @param data Pointer to the data to be stored
 */
void nvm_logger_feed_data (uint32_t log_id, void * data);
//
/**
 * @brief Function to get last data entry.
 * @param log_id Log ID of log from where you want to get last data.
 * @param dest_loc Pointer to the destination location where this data is to be stored
 */
//void nvm_logger_get_last_data (uint32_t log_id, void * dest_loc);
//
/**
 * @breif Function to get all the data entries from log
 * @param log_id Log ID of 
 */
//void nvm_logger_get_all_data (uint32_t log_id, void * dest_loc);

/**
 * @brief Function to get n number of data entries from log
 * @param log_id Log ID of log from which data is required
 * @param dest_loc Pointer to the destination location where data is to be stored
 * @param n Number of data entries that are needed to be retrieved.
 * @Note For now direction is disable. it'll take data from last to first
 * @param dir Direction in which data is to be retrieved.
 */
void nvm_logger_get_n_data (uint32_t log_id, void * dest_loc, uint32_t n);

/**
 * @Brief Function to fetch specific entry from last
 * @param log_id Log ID of log from which data is to be fetched
 * @param dest_loc Pointer to location where data is to be stored
 * @param entry_no Entry number from end.
 */
void nvm_logger_fetch_tail_data (uint32_t log_id, void * dest_loc, uint32_t entry_no);

/**
 * @brief Function to empty the log
 * @param log_id Log ID of log which is to be emptied
 */
void nvm_logger_empty_log (uint32_t log_id);

/**
 * @brief Function to check if log is empty or not.
 * @param log_id Log ID of log which is to be checked.
 * @return Return the status of log.
 * @retval true Log is empty
 * @retval false Log is not empty
 */
bool nvm_logger_is_log_empty (uint32_t log_id);

/**
 * @brief Function to release the log. Once released, page used by that log will\
 * be erased and available for other logs to use.
 * @param log_id Log ID of log which is to be released.
 * @Note Two logs cannot access the same log page.
 */
void nvm_logger_release_log (uint32_t log_id);

uint32_t nvm_logger_get_total_entries (uint32_t log_id);
#endif /* NVM_LOGGER_H */
