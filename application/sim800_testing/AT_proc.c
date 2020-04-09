/*
 *  AT_proc.c : <Write brief>
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

typedef enum 
{
    CMD_SUCCESSFUL,
    CMD_RUNNING,
    CMD_REPEAT,
    CMD_FAILED,
}cmd_status_t;

static uint8_t response[HAL_UARTE_RX_BUFF_SIZE];

uint8_t g_arr_rsp[AT_PROC_MAX_RESPOSES][HAL_UARTE_RX_BUFF_SIZE];
uint8_t g_arr_err[AT_PROC_MAX_ERRORS][HAL_UARTE_RX_BUFF_SIZE];

static uint8_t g_arr_cmd[HAL_UARTE_TX_BUFF_SIZE];

static uint8_t g_cmd_len = 0;

static at_proc_cmd_t g_buff_cmd;

static bool cmd_is_critical = false;

static bool rsp_is_var = false;

static cmd_status_t g_current_status = CMD_FAILED; 

volatile bool mod_is_busy = false;

volatile uint32_t g_timeout_ticks;

volatile uint32_t g_current_ticks;


void (* cmd_successful_handle) (uint32_t response_id);
//void (* cmd_successful_data_handle) (at_uart_data_t data1, at_uart_data_t data2);
void (* cmd_failed_handle) (uint8_t is_critical, uint8_t is_timeout, uint32_t error_id);

void collect_rsp (uint8_t rsp_char);


void stop_uart ()
{
    hal_uarte_stop_rx ();
}

void handle_critical ()
{
    if((cmd_is_critical) && (g_current_status != CMD_SUCCESSFUL))
    {
        mod_is_busy = true;
        
    }
}

void ticks_reset ()
{
    g_current_ticks = 0;
}

void timeout_handler ()
{
//    stop_uart ();
    mod_is_busy = false;
    handle_critical ();
    cmd_failed_handle (1, cmd_is_critical, AT_PROC_MAX_ERRORS);

}

void chk_rsp ()
{
    for(uint32_t rsp_cnt = 0; rsp_cnt < AT_PROC_MAX_RESPOSES; rsp_cnt++)
    {
        if (strcmp ((char * )response, (char *)&g_arr_rsp[rsp_cnt][0]) == 0)
        {
//            stop_uart ();
            mod_is_busy = false;
            log_printf("Rsp : %d\n", rsp_cnt);
            g_current_status = CMD_SUCCESSFUL;
            cmd_successful_handle (rsp_cnt);
            break;
        }
    }
}

void chk_err ()
{
    for(uint32_t err_cnt = 0; err_cnt < AT_PROC_MAX_ERRORS; err_cnt++)
    {
        if(strcmp ((char *)response, (char *)&g_arr_err[err_cnt][0]) == 0)
        {
            log_printf("Err : %d\n", err_cnt);
            if(cmd_is_critical)
            {
//                stop_uart ();
                mod_is_busy = false;
                handle_critical ();
                g_current_status = CMD_REPEAT;
            }
            else
            {
//                stop_uart ();
                mod_is_busy = false;
                cmd_failed_handle (0, cmd_is_critical, err_cnt);
                g_current_status = CMD_FAILED;
                break;
            }
        }
    }
}


void handle_stray ()
{
}

void fix_rsp ()
{
    chk_rsp ();
    chk_err ();
}

void var_rsp ()
{
}

void process_rsp ()
{  
    log_printf ("Status : %d\n", g_current_status);
    if((g_current_status == CMD_RUNNING) || (g_current_status == CMD_REPEAT))
    {
        log_printf ("Process\n");
        if(rsp_is_var)
        {
            var_rsp ();
        }
        else
        {
            fix_rsp ();
        }
    }
    else
    {
        log_printf ("Skip\n");
    }
}

void collect_rsp (uint8_t rsp_char)
{
    static uint8_t len = 0;
    static uint8_t l_prev_char;
    response[len] = rsp_char;
    len++;
    log_printf ("%c", rsp_char);
    if((l_prev_char == '\r') && (rsp_char == '\n'))
    {
        log_printf("%s\n",(char *)response);
        process_rsp ();
        memset (response, 0, sizeof(response));
        len=0;
    }

    l_prev_char = rsp_char;

}

void AT_proc_init (AT_proc_init_t * init)
{
    hal_uarte_init (HAL_UARTE_BAUD_9600, APP_IRQ_PRIORITY_MID);
    cmd_successful_handle = init->cmd_successful;
    cmd_failed_handle = init->cmd_failed;
}


at_proc_cmd_check_t AT_proc_send_cmd (at_proc_cmd_t * cmd)
{
    memcpy (&g_buff_cmd, cmd, sizeof(at_proc_cmd_t));
    memset (g_arr_rsp, 0, sizeof(g_arr_rsp));
    memset (g_arr_err, 0, sizeof(g_arr_err));
    
    g_cmd_len = cmd->cmd.len;
    
    memcpy (g_arr_cmd, cmd->cmd.ptr, cmd->cmd.len);
    
    for(uint32_t cnt = 0; cnt < AT_PROC_MAX_RESPOSES; cnt++)
    {
        if((cmd->resp[cnt].ptr != NULL) && (cmd->resp[cnt].len < HAL_UARTE_TX_BUFF_SIZE))
        {
            memcpy (&g_arr_rsp[cnt][0], cmd->resp[cnt].ptr, cmd->resp[cnt].len);
        }
    }
    
    for(uint32_t cnt = 0; cnt < AT_PROC_MAX_ERRORS; cnt++)
    {
        if((cmd->err[cnt].ptr != NULL) && (cmd->err[cnt].len < HAL_UARTE_TX_BUFF_SIZE))
        {
            memcpy (&g_arr_err[cnt][0], cmd->err[cnt].ptr, cmd->err[cnt].len);
        }
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
    hal_uarte_puts (cmd,len);
}

void AT_proc_add_ticks (uint32_t ticks)
{
    if ((mod_is_busy))
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
                log_printf ("Here\n");
                {
                    g_current_status = CMD_RUNNING;
                    hal_uarte_start_rx (collect_rsp);
                    hal_uarte_puts (g_arr_cmd, g_cmd_len);
                }
                break;
            }
            case CMD_FAILED : 
                break;
        }
    }
}

void AT_proc_repeat_last_cmd ()
{
    g_current_status = CMD_REPEAT;
}

