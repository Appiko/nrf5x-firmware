/*
 *  AT_proc.c : Module to process AT commands
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
#include "AT_proc.h"

#include "stdbool.h"
#include "hal_uarte.h"
#include "log.h"
#include "string.h"
#include "nrf_util.h"
#include "ms_timer.h"
#include "hal_nop_delay.h"

/** List of all possiable states for command */
typedef enum 
{
    CMD_SUCCESSFUL,
    CMD_RUNNING,
    CMD_REPEAT,
    CMD_FAILED,
}cmd_status_t;

/** Buffer to store data received over uart */
static uint8_t response[HAL_UARTE_RX_BUFF_SIZE];
/** Multiline buffer to store all expected responses */
uint8_t g_arr_rsp[AT_PROC_MAX_RESPOSES][HAL_UARTE_RX_BUFF_SIZE];
/** Multiline buffer to store all expected errors*/
uint8_t g_arr_err[AT_PROC_MAX_ERRORS][HAL_UARTE_RX_BUFF_SIZE];
/** Array to store uart_data received for command whose response is not well defined */
at_uart_data_t g_arr_at_rsp[AT_PROC_MAX_RESPOSES + AT_PROC_MAX_ERRORS];
/** Variable to store number of lines receievd as a part of variable response */
static uint32_t g_var_rsp_lcnt = 0;
/** Buffer to store AT command which is to be execcuated */
static uint8_t g_arr_cmd[HAL_UARTE_TX_BUFF_SIZE];
/** Variable to store length of current AT command */
static uint8_t g_cmd_len = 0;
/** Variable to store command ID of current command */
static uint32_t g_cmd_id;
/** Variable to store a flag to check if command is critical */
static bool cmd_is_critical = false;
/** Variable to store a flag to check if variable response is expected */
static bool rsp_is_var = false;
/** Variable to store current status of command which is being executed */
static cmd_status_t g_current_status = CMD_FAILED; 
/** Flag which stores modules status */
volatile bool mod_is_busy = false;
/** Variable to store timeout duration in ticks for current command */
volatile uint32_t g_timeout_ticks;
/** Variable to store current ticks */
volatile uint32_t g_current_ticks;

/** Function pointer to the function which is to be called when command is successfully executed */
void (* cmd_successful_handle) (uint32_t cmd_id, uint32_t response_id);
/** Function pointer to the function which is to be called when unknown response in received */
void (* cmd_successful_data_handle) (uint32_t cmd_id, at_uart_data_t * u_data1, uint32_t len);
/** Function pointer to the function which is to be called when command execution fails */
void (* cmd_failed_handle) (uint32_t cmd_id, uint8_t is_critical, uint8_t is_timeout, uint32_t error_id);

/** Local supporting fucntions */

/**
 * @brief Function to handle critical command failure
 */
void handle_critical ()
{
    if((cmd_is_critical) && (g_current_status != CMD_SUCCESSFUL))
    {
        mod_is_busy = true;
        
    }
}

/**
 * @brief Function to reset current ticks
 */
void ticks_reset ()
{
    g_current_ticks = 0;
}

/**
 * @brief Function to handle timeout event
 */
void timeout_handler ()
{
    mod_is_busy = false;
    if (rsp_is_var)
    {
        g_current_status = CMD_SUCCESSFUL;
        cmd_successful_data_handle (g_cmd_id, g_arr_at_rsp, g_var_rsp_lcnt);
        g_var_rsp_lcnt = 0;
    }
    else
    {
        handle_critical ();
        cmd_failed_handle (g_cmd_id, 1, cmd_is_critical, AT_PROC_MAX_ERRORS);
    }

}

/**
 * @brief Functiont to comapre response with responses
 */
void chk_rsp ()
{
    for(uint32_t rsp_cnt = 0; rsp_cnt < AT_PROC_MAX_RESPOSES; rsp_cnt++)
    {
        if (strcmp ((char * )response, (char *)&g_arr_rsp[rsp_cnt][0]) == 0)
        {
            mod_is_busy = false;
            g_current_status = CMD_SUCCESSFUL;
            cmd_successful_handle (g_cmd_id, rsp_cnt);
            break;
        }
    }
}

/**
 * @brief Function to comapre response with errors
 */
void chk_err ()
{
    for(uint32_t err_cnt = 0; err_cnt < AT_PROC_MAX_ERRORS; err_cnt++)
    {
        if(strcmp ((char *)response, (char *)&g_arr_err[err_cnt][0]) == 0)
        {
            if(cmd_is_critical)
            {
                mod_is_busy = false;
                handle_critical ();
                g_current_status = CMD_REPEAT;
            }
            else
            {
                mod_is_busy = false;
                cmd_failed_handle (g_cmd_id, 0, cmd_is_critical, err_cnt);
                g_current_status = CMD_FAILED;
                break;
            }
        }
    }
}

/**
 * @brief Function to store expected responses for current command into a global buffer
 * @param cmd Strcture pointer to structure storing current command
 */
void set_rsps (at_proc_cmd_t * cmd)
{
    for(uint32_t cnt = 0; cnt < AT_PROC_MAX_RESPOSES; cnt++)
    {
        if((cmd->resp[cnt].ptr != NULL) && (cmd->resp[cnt].len < HAL_UARTE_TX_BUFF_SIZE))
        {
            memcpy (&g_arr_rsp[cnt][0], cmd->resp[cnt].ptr, cmd->resp[cnt].len);
        }
    }
}

/**
 * @brief Function to store expected errors for current command into a gloabal buffer
 * @param cmd Strucutre pointer to structure storing current command
 */
void set_errs (at_proc_cmd_t * cmd)
{
    
    for(uint32_t cnt = 0; cnt < AT_PROC_MAX_ERRORS; cnt++)
    {
        if((cmd->err[cnt].ptr != NULL) && (cmd->err[cnt].len < HAL_UARTE_TX_BUFF_SIZE))
        {
            memcpy (&g_arr_err[cnt][0], cmd->err[cnt].ptr, cmd->err[cnt].len);
        }
    }
}

/**
 * @brief Function to set global buffer as a data buffer to store variable response.
 */
void set_data_buff ()
{
    for (uint32_t cnt_first_h = 0; cnt_first_h < AT_PROC_MAX_RESPOSES; cnt_first_h++)
    {
        g_arr_at_rsp[cnt_first_h].ptr = (char *)&g_arr_rsp[cnt_first_h][0];
    }
    for (uint32_t cnt_sec_h = 0; cnt_sec_h < AT_PROC_MAX_ERRORS; cnt_sec_h++)
    {
        g_arr_at_rsp[(cnt_sec_h + AT_PROC_MAX_RESPOSES)].ptr = 
            (char *)&g_arr_err[cnt_sec_h][0];
    }
}

/**
 * @brief Function to handle response when definate response is exepected
 */
void fix_rsp_handler ()
{
    chk_rsp ();
    chk_err ();
}

/**
 * @brief Function to handle response when variable response is exepected
 */
void var_rsp_handler ()
{
    g_arr_at_rsp[g_var_rsp_lcnt].len = strlen ((char *)response);
    if (g_var_rsp_lcnt < (AT_PROC_MAX_RESPOSES + AT_PROC_MAX_ERRORS))
    {
        strcpy (g_arr_at_rsp[g_var_rsp_lcnt].ptr, (char *)response);
    }
    else
    {
        cmd_successful_data_handle (g_cmd_id, g_arr_at_rsp, g_var_rsp_lcnt);
        g_var_rsp_lcnt = 0;
    }
    g_var_rsp_lcnt++;
}

/**
 * @brief Function to process the response
 */
void process_rsp ()
{  
    if((g_current_status == CMD_RUNNING) || (g_current_status == CMD_REPEAT))
    {
        if(rsp_is_var)
        {
            var_rsp_handler ();
        }
        else
        {
            fix_rsp_handler ();
        }
    }
    else
    {
    }
}

/**
 * @brief Function to collect characters to generate response
 * @param rsp_char First character in UART buffer
 */
void collect_rsp (uint8_t rsp_char)
{
    static uint8_t len = 0;
    static uint8_t l_prev_char;
    response[len] = rsp_char;
    len++;
    if((l_prev_char == '\r') && (rsp_char == '\n'))
    {
        process_rsp ();
        memset (response, 0, sizeof(response));
        len=0;
    }

    l_prev_char = rsp_char;

}

/** Refer AT_proc.h */

void AT_proc_init (AT_proc_init_t * init)
{
    hal_uarte_init (HAL_UARTE_BAUD_9600, APP_IRQ_PRIORITY_MID);
    cmd_successful_handle = init->cmd_successful;
    cmd_failed_handle = init->cmd_failed;
    cmd_successful_data_handle = init->cmd_successful_data;
}


at_proc_cmd_check_t AT_proc_send_cmd (at_proc_cmd_t * cmd)
{
    memset (g_arr_rsp, 0, sizeof(g_arr_rsp));
    memset (g_arr_err, 0, sizeof(g_arr_err));
    
    g_cmd_len = cmd->cmd.len;
    
    g_cmd_id = cmd->cmd_id;
    
    memcpy (g_arr_cmd, cmd->cmd.ptr, cmd->cmd.len);
    
    if(cmd->is_response_variable)
    {
        set_data_buff ();
    }
    else
    {
        set_rsps (cmd);
        set_errs (cmd);
    }
    
    ticks_reset ();
    g_timeout_ticks = MS_TIMER_TICKS_MS (cmd->timeout);
    cmd_is_critical = cmd->is_critical;
    rsp_is_var = cmd->is_response_variable;
    g_current_status = CMD_RUNNING;
    hal_uarte_start_rx (collect_rsp);
    hal_uarte_puts ((uint8_t * )cmd->cmd.ptr, cmd->cmd.len);
    mod_is_busy = true;
    return AT_CMD_OK;
}

void AT_proc_process ()
{
    hal_uarte_process ();
}


uint8_t AT_proc_is_busy ()
{
    return (uint8_t)mod_is_busy;
}

void AT_proc_send_cmd_no_rsp (uint8_t * cmd, uint32_t len, uint32_t duration)
{
    mod_is_busy = true;
    g_current_status = CMD_SUCCESSFUL;
    hal_uarte_puts (cmd,len);
    hal_nop_delay_ms (duration);
    mod_is_busy = false;
}

void AT_proc_add_ticks (uint32_t ticks)
{
    g_current_ticks += ticks;
    switch(g_current_status)
    {
        case CMD_SUCCESSFUL :
            break;
        case CMD_RUNNING : 
        {
            if (g_timeout_ticks <= g_current_ticks)
            {
                ticks_reset ();
                g_current_status = CMD_FAILED;
                timeout_handler ();
            }
            break;
        }
        case CMD_REPEAT : 
        {
            {
                g_current_status = CMD_RUNNING;
                // uncomment this if UART is being disabled after command execution is done 
                //hal_uarte_start_rx (collect_rsp);
                hal_uarte_puts (g_arr_cmd, g_cmd_len);
            }
            break;
        }
        case CMD_FAILED : 
            break;
    }
}

void AT_proc_repeat_last_cmd ()
{
    g_current_status = CMD_REPEAT;
}

