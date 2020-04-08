/*
 *  sim800_oper.c : <Write brief>
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


#include "sim800_oper.h"
#include "AT_proc.h"
#include "log.h"


#include "stdbool.h"
#include "string.h"


/** @brief Size of the ring buffer for the message list
 *  @note Should be a power of 2 */
#define ATbuff_SIZE  64

#include "CBUF.h"
#include "hal_nop_delay.h"
/** Check if MSG_SIZE is power of 2 */
#if (!(!(ATbuff_SIZE & (ATbuff_SIZE-1)) && ATbuff_SIZE))
#error ATbuff_SIZE must be a power of 2
#endif

#define MAX_LEN_CMD 128



volatile static struct
{
    uint32_t m_getIdx;
    uint32_t m_putIdx;
    at_proc_cmd_t m_entry[ATbuff_SIZE];
} ATbuff;


static char rsp_std_OK[] = {'O','K','\r','\n'};
static char rsp_std_ERR[] = {'E','R','R','O','R','\r','\n'};

/**
 * Init Seq
 * AT -- OK -- ERROR
 * AT+CLTS=1 -- 
 * AT&W -- 
 * AT+CFUN=0 -- 
 * AT+CFUN=1,1 -- 
 * AT&FZ -- 
 * ATE0 -- 
 * AT+CPIN? -- 
 */
static char cmd_init[] = {'A','T','\r','\n'}; //Res : std reply
static char cmd_fac_reset[] = {'A','T','&','F','Z','\r','\n'};
static char cmd_echo_off[] = {'A','T','E','0','\r','\n'}; //Res : ATE0 --std reply
static char cmd_time_fmt[] = {'A','T','+','C','L','T','S','=','1','\r','\n'}; //Res : std reply
static char cmd_save_nvm[] = {'A','T','&','W','\r','\n'};
static char cmd_en_full[] = {'A','T','+','C','F','U','N','=','1',',','1','\r','\n'}; //Res : std reply

static char cmd_chk_sim[] = {'A','T','+','C','P','I','N','?','\r','\n'};
static char rsp_sim_sts1[] = {'+','C','P','I','N',':',' ','R','E','A','D','Y','\r','\n'};
static char err_sim_sts1[] = {'+','C','P','I','N',':',' ','E','R','R','O','R','\r','\n'};

/**
 * Check for network
 * AT+CREG? -- 
 */
const char cmd_gprs_reg[] = {'A','T','+','C','G','R','E','G','?','\r','\n'};

const char rsp1l2_gprs_reg[] = {'+','C','G','R','E','G',':',' ','0',',','1','\r','\n'};
const char rsp2l2_gprs_reg[] = {'+','C','G','R','E','G',':',' ','0',',','5','\r','\n'};

const char err1l2_gprs_reg[] = {'+','C','G','R','E','G',':',' ','0',',','0','\r','\n'};
const char err2l2_gprs_reg[] = {'+','C','G','R','E','G',':',' ','0',',','2','\r','\n'};
const char err3l2_gprs_reg[] = {'+','C','G','R','E','G',':',' ','0',',','4','\r','\n'};

/**
 * Connect to GPRS
 * AT+SAPBR=3,1,"Contype","GPRS" -- 
 * AT+SAPBR=3,1,"APN","www" -- 
 * AT+SAPBR=3,1,"USER","" -- 
 * AT+SAPBR=3,1,"PWD","" -- 
 * 
 * AT+CGDCONT=1,"IP","www" -- 
 * AT+CGACT=1,1 -- 
 * AT+SAPBR=1,1 --
 * AT+SAPBR=2,1 --
 * AT+CGATT=1 --
 * AT+CIPMUX=1 --
 * AT+CIPQSEND=1 --
 * AT+CIPRXGET=1 -- 
 * AT+CSTT="www","","" -- 
 * AT+CIICR --
 * AT+CIFSR -- 
 * AT+CDNSCFG="8.8.8.8","8.8.4.4" -- 
 */

//
//const char cmd_set_brr_ctype[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
//    '\"','C','o','n','t','y','p','e','\"',',','\"','G','P','R','S','\"','\r','\n'}; //Res : std
////static char cmd_set_brr_apn[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
////    '\"','A','P','N','\"',',','\"','w','w','w','\"','\r','\n'};//Res : std
//const char cmd_set_brr_usr[]= {'A','T','+','S','A','P','B','R','=','3',',','1',',',
//    '\"','U','S','E','R','\"',',','\"','\"','\r','\n'};//Res: std
//const char cmd_set_brr_pwd[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
//    '\"','P','W','D','\"',',','\"','\"','\r','\n'};//Res: std
//
////static char cmd_ip_cntxt[] = {'A','T','+','C','G','D','C','O','N','T','=','1',',',
////    '\"','I','P','\"',',','\"','a','p','n','\"','\r','\n'};
//
//
//const char cmd_actv_cid[] = {'A','T','+','C','G','A','C','T','=',
//'1',',','1','\r','\n'};//res: Std ok, CME error
//
//const char cmd_actv_brr[] = {'A','T','+','S','A','P','B','R','=','1',',','1','\r','\n'};
//const char cmd_chk_brr[] = {'A','T','+','S','A','P','B','R','=','2',',','1','\r','\n'}; //res: variable
//
//const char cmd_en_mux[] =  {'A','T','+','C','I','P','M','U','X','=','1','\r','\n'};
//
//const char cmd_sel_dtx_mode[] = {'A','T','+','C','I','P','Q','S','E','N','D','=','1','\r','\n'};
//
//const char cmd_get_net_data[] = {'A','T','+','C','I','P','R','X','G','E','T','=','1','\r','\n'};
//
//const char cmd_tsk_start[] = {'A','T','+','C','S','T','T','=','\"','w','w','w','\"',',','\"','\"',',','\"','\"','\r','\n'};
//
//const char cmd_con_gprs[] = {'A','T','+','C','I','I','C','R','\r','\n'};
//
//const char cmd_get_ip[] = {'A','T','+','C','I','F','S','R','\r','\n'};
//
//const char cmd_set_dns[] = {'A','T','+','C','D','N','S','C','F','G','=',
//'\"','8','.','8','.','8','.','8','\"',',','\"','8','.','8','.','4','.','4','\"','\r','\n'};
/**
 * In this module we will have queue of AT_proc_cmds. 
 * On each init or enable call, we'll add appropriate AT commands to that queue
 */


void at_process ()
{
    //Do at process
}

void network_check_process ()
{
    //Add check for network
}

void send_next_cmd ()
{
    if (CBUF_Len(ATbuff))
    {
        at_proc_cmd_t l_cmd;
        l_cmd = CBUF_Pop(ATbuff);
        AT_proc_send_cmd (&l_cmd);
    }
}

void reset_cmd (at_proc_cmd_t * cmd)
{
    memset (cmd, 0, sizeof(at_proc_cmd_t));
}

void command_processed_successfully (uint32_t rsp_id)
{
    log_printf("%s\n",__func__);
//    send_next_cmd ();
}

void command_unknown_response (at_uart_data_t u_data1, at_uart_data_t u_data2)
{
    log_printf("%s\n",__func__);
    
}

void command_process_failure (uint8_t was_critical, uint8_t was_timeout,uint32_t error_id)
{
    log_printf("%s\n",__func__);
    
    if (was_critical == false)
    {
//        send_next_cmd ();
    }
}

AT_proc_init_t at_init = 
{
    .cmd_successful = command_processed_successfully,
    .cmd_successful_data = command_unknown_response,
    .cmd_failed = command_process_failure,
};


void sim800_oper_init (sim800_operator_t oper)
{
    CBUF_Init(ATbuff);
    AT_proc_init (&at_init);
    //Add Init Seq to command buffer
    at_proc_cmd_t l_at_cmd;
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_init;
    l_at_cmd.cmd.len = sizeof(cmd_init);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 1;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_fac_reset;
    l_at_cmd.cmd.len = sizeof(cmd_fac_reset);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_echo_off;
    l_at_cmd.cmd.len = sizeof(cmd_echo_off);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_time_fmt;
    l_at_cmd.cmd.len = sizeof(cmd_time_fmt);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_save_nvm;
    l_at_cmd.cmd.len = sizeof(cmd_save_nvm);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);

    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_chk_sim;
    l_at_cmd.cmd.len = sizeof(cmd_chk_sim);
    l_at_cmd.resp[0].ptr = rsp_sim_sts1;
    l_at_cmd.resp[0].len = sizeof(rsp_sim_sts1);
    l_at_cmd.err[0].ptr = err_sim_sts1;
    l_at_cmd.err[0].len = sizeof(err_sim_sts1);
    l_at_cmd.is_critical = 1;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = cmd_en_full;
    l_at_cmd.cmd.len = sizeof(cmd_en_full);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);

    
}

void sim800_oper_full_init (sim800_operator_t oper)
{
    sim800_oper_init (oper);
}

void sim800_oper_enable_sms (void)
{
}

void sim800_oper_enable_gprs (void)
{
    //Add GPRS
}

/**
 * And in process function those AT commands will get exec one after another. 
 */
void sim800_oper_process ()
{
//    log_printf("SIM800 Proc : ");
    at_proc_cmd_t l_cmd;
    if(AT_proc_is_busy ())
    {
//        log_printf("Process\n");
        AT_proc_process ();
    }
    else if(CBUF_Len(ATbuff))
    {
//        log_printf("Cmd\n");
        l_cmd = CBUF_Pop(ATbuff);
        AT_proc_send_cmd (&l_cmd);
    }
    else
    {
//        log_printf("Return\n");
        return;
    }
    return;
    //while checking for network, try until check_time < (macro)MAX_NETWORK_TIMEOUT
    //if (chk_nw_flag == true) && (flag_network_detected == false) &&  (check_time < MAX_NETWORK_TIMEOUT)
        //check network
    //else
        //execute command
}

void sim800_oper_add_ticks (uint32_t ticks)
{
    //pass on these ticks to AT_proc
    AT_proc_add_ticks (ticks);
}

uint32_t sim800_oper_conns (sim800_server_conn_t * conn_params)
{
    return 0;
}

void sim800_oper_http_req (sim800_http_req_t * http_req)
{
}

sim800_conn_status_t sim800_oper_get_gprs_status ()
{
    return SIM800_CONNECTED;
}

sim800_conn_status_t sim800_oper_get_server_status (uint32_t server_id)
{
    return SIM800_CONNECTED;
}



