/*
 *  AT_proc.h : Module to process AT commands
 *  Copyright (C) 2020  Appiko
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
 * @addtogroup group_at_lib
 * @{
 *
 * @defgroup group_at_proc AT Process
 * @brief Driver to Process AT commands using hal_uarte module
 *
 * @warning This module needs the device_tick and to be on and running to be able to work
 * @note This module uses UARTE peripheral. So developer can't use uart prints for debugging
 * @{
 */

#ifndef AT_PROC_H
#define AT_PROC_H

#include "stdint.h"

#ifndef AT_PROC_MULTILINE_LINES
#define AT_PROC_MULTILINE_LINES 3
#endif

#ifndef AT_PROC_MAX_RESPOSES
#define AT_PROC_MAX_RESPOSES 4 
#endif

#ifndef AT_PROC_MAX_ERRORS
#define AT_PROC_MAX_ERRORS 4 
#endif 
/** AT command sanity result */
typedef enum 
{
    /** AT command terminated properly */
    AT_CMD_OK,
    /** AT command not terminated properly */
    AT_CMD_INVALID,
            
}at_proc_cmd_check_t;

/** Structure to store single line command or response */
typedef struct
{
    /** Pointer to the string */
    char * ptr;
    /** Length of string */
    uint32_t len;
}at_uart_data_t;


///** Structure to store multi-line command or response */
//typedef struct
//{
//    /** Array of pointers */
//    char ptrs[][AT_PROC_MULTILINE_LINES];
//    /** Length of each string */
//    uint8_t len[AT_PROC_MULTILINE_LINES];
//}at_uart_multiline_t;

typedef struct 
{
    /** Command ID */
    uint32_t cmd_id;
    
    /** Command + parameters*/
    at_uart_data_t cmd;

    /** Error */
    at_uart_data_t err[AT_PROC_MAX_ERRORS];

    /** Response */
    at_uart_data_t resp[AT_PROC_MAX_RESPOSES];
    
    
    /** Timeout */
    uint32_t timeout;
    
    /** Is critical */
    uint8_t is_critical;
    
    /** Is response variable */
    uint8_t is_response_variable;
    
}at_proc_cmd_t;

/** Structure to store different function pointers */
typedef struct
{
    /** 
     * Function pointer of function which is to be called when cmd is executed successfully
     * known response.
     */
    void (* cmd_successful) (uint32_t cmd_id, uint32_t response_id);
    
    /** 
     * Function pointer of function which is to be called when cmd is executed but 
     * response is unknown
     */
    void (* cmd_successful_data) (uint32_t cmd_id, at_uart_data_t * u_data1, uint32_t len);
    
    /**
     * Function pointer to function which is to be called when cmd is failed
     */
    void (* cmd_failed) (uint32_t cmd_id, uint8_t is_critical, uint8_t is_timeout, uint32_t error_id);
    
}AT_proc_init_t;


void AT_proc_init (AT_proc_init_t * init);

/**
 * @brief Function to send AT command.
 * @param cmd Structure pointer to structure which stores a AT command parameter.
 * @return Return AT_CMD_INVALID if AT command is not terminated properly
 */
at_proc_cmd_check_t AT_proc_send_cmd (at_proc_cmd_t * cmd);

/**
 * @brief Function to handle add ticks event for AT command process module
 * @param ticks Number of ticks since last add_tick event
 */
void AT_proc_add_ticks (uint32_t ticks);

/**
 * @brief Function to handle event and data generated at interrupt level at thread level
 * @note it has to be called in dependent module's process function
 */
void AT_proc_process ();

/**
 * @brief Function to check if AT command process module is busy
 * @return 1 if module is executing some AT command
 * @return 0 if module is available
 */
uint8_t AT_proc_is_busy ();

/**
 * @brief Function to repeat the last instruction with same parameters.
 */
void AT_proc_repeat_last_cmd ();

/**
 * @brief Function to execute an AT command where response is not needed.
 * @param cmd Command string
 * @param len Length of command string
 * @param duration Duration for which module will be marked busy.
 * @note This function don't care if operation was successful or not.
 */
void AT_proc_send_cmd_no_rsp (uint8_t * cmd, uint32_t len, uint32_t duration);

#endif /* AT_PROC_H */
/**
 * @}
 * @}
 */
