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
#include "sim800_cmd_id.h"
#include "log.h"


#include "stdbool.h"
#include "string.h"


/** @brief Size of the ring buffer for the message list
 *  @note Should be a power of 2 */
#define ATbuff_SIZE  64

#include "CBUF.h"
#include "hal_nop_delay.h"
#include "ms_timer.h"
/** Check if MSG_SIZE is power of 2 */
#if (!(!(ATbuff_SIZE & (ATbuff_SIZE-1)) && ATbuff_SIZE))
#error ATbuff_SIZE must be a power of 2
#endif

#define MAX_LEN_CMD 128
#define MAX_LEN_RSP 512



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

#define NET_CHK_FREQ    (MS_TIMER_TICKS_MS(10000))

const char cmd_init[] = {'A','T','\r','\n'}; //Res : std reply
const char cmd_fac_reset[] = {'A','T','&','F','Z','\r','\n'};
//static char cmd_echo_off[] = {'A','T','E','0','\r','\n'}; //Res : ATE0 --std reply
const char cmd_time_fmt[] = {'A','T','+','C','L','T','S','=','1','\r','\n'}; //Res : std reply
const char cmd_save_nvm[] = {'A','T','&','W','\r','\n'};

const char cmd_dis_full[] = {'A','T','+','C','F','U','N','=','0','\r','\n'}; //Res : std reply

const char cmd_en_full[] = {'A','T','+','C','F','U','N','=','1','\r','\n'}; //Res : std reply

const char cmd_chk_sim[] = {'A','T','+','C','P','I','N','?','\r','\n'};
const char rsp_sim_sts1[] = {'+','C','P','I','N',':',' ','R','E','A','D','Y','\r','\n'};
const char err_sim_sts1[] = {'+','C','P','I','N',':',' ','E','R','R','O','R','\r','\n'};

/**
 * Check for network
 * AT+CREG? -- 
 */
const char cmd_nw_reg[] = {'A','T','+','C','R','E','G','?','\r','\n'};

const char rsp1l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','1','\r','\n'};
const char rsp2l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','5','\r','\n'};

const char err1l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','0','\r','\n'};
const char err2l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','2','\r','\n'};
const char err3l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','4','\r','\n'};


const char cmd_chk_net[] = {'A','T','+','C','G','A','T','T','?','\r','\n'};

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
const char cmd_set_brr_ctype[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','C','o','n','t','y','p','e','\"',',','\"','G','P','R','S','\"','\r','\n'}; //Res : std
static char cmd_set_brr_apn[MAX_LEN_CMD];

const char cmd_set_brr_usr[]= {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','U','S','E','R','\"',',','\"','\"','\r','\n'};//Res: std
const char cmd_set_brr_pwd[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','P','W','D','\"',',','\"','\"','\r','\n'};//Res: std

static char cmd_ip_cntxt[MAX_LEN_CMD];

const char cmd_power_off[] = {'A','T','+','C','P','O','W','D','=','1','\r','\n'};


const char cmd_actv_cid[] = {'A','T','+','C','G','A','C','T','=',
'1',',','1','\r','\n'};//res: Std ok, CME error

const char cmd_actv_brr[] = {'A','T','+','S','A','P','B','R','=','1',',','1','\r','\n'};
const char cmd_chk_brr[] = {'A','T','+','S','A','P','B','R','=','2',',','1','\r','\n'}; //res: variable

const char cmd_en_mux[] =  {'A','T','+','C','I','P','M','U','X','=','1','\r','\n'};

const char cmd_sel_dtx_mode[] = {'A','T','+','C','I','P','Q','S','E','N','D','=','1','\r','\n'};

//static char cmd_get_net_data[] = {'A','T','+','C','I','P','R','X','G','E','T','=','1','\r','\n'};

static char cmd_tsk_start[MAX_LEN_CMD];

const char cmd_con_gprs[] = {'A','T','+','C','I','I','C','R','\r','\n'};

const char cmd_get_ip[] = {'A','T','+','C','I','F','S','R','\r','\n'};

const char cmd_set_dns[] = {'A','T','+','C','D','N','S','C','F','G','=',
'\"','8','.','8','.','8','.','8','\"',',','\"','8','.','8','.','4','.','4','\"','\r','\n'};


const char cmd_http_init[] = {'A','T','+','H','T','T','P','I','N','I','T','\r','\n'};

const char cmd_http_cid[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
'C','I','D','\"',',','1','\r','\n'};

const char cmd_http_type[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
'C','I','D','\"',',','1','\r','\n'};

static char cmd_http_url[MAX_LEN_CMD];
//const char cmd_http_url[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
//'U','R','L','\"',',','\"','w','w','w','.','s','i','m','.','c','o','m','\"','\r','\n'};

const char cmd_http_get[] = {'A','T','+','H','T','T','P','A','C','T','I','O','N',
'=','0','\r','\n'};

const char cmd_http_post[] = {'A','T','+','H','T','T','P','A','C','T','I','O','N',
'=','1','\r','\n'};

static char cmd_http_set_pdata[MAX_LEN_CMD];
const char rsp_http_set_pdata[] = {'D','O','W','N','L','O','A','D','\n'};
//static char cmd_http_set_pdata[] = {'A','T','+','H','T','T','P','D','A','T','A','=','7',',','7','0','0','0','\r','\n'};

const char cmd_http_read[] = {'A','T','+','H','T','T','P','R','E','A','D','\r','\n'};

const char cmd_http_term[] = {'A','T','+','H','T','T','P','T','E','R','M','\r','\n'};

static char data_http_post[MAX_LEN_CMD];

static uint32_t http_pdata_len = 0;

//static uint32_t g_rerun_cnt = 0;
//
//static uint8_t g_rx_data[MAX_LEN_RSP];



/**
 * In this module we will have queue of AT_proc_cmds. 
 * On each init or enable call, we'll add appropriate AT commands to that queue
 */

typedef enum
{
    NET_LOST,
    NET_SEARCHING,
    NET_FOUND,
}network_status_t;


typedef enum
{
    CRITIC_MOD_INIT     = SIM800_MOD_INIT,
    CRITIC_MOD_CHK_SIM  = SIM800_MOD_CHK_SIM,
    CRITIC_NET_CHK      = SIM800_NET_CHK,
    CRITIC_NET_EN_GPRS  = SIM800_NET_EN_GPRS,            
}critical_cmd;

typedef enum
{
    INFO_NET_BCHK       = SIM800_NET_BCHK,
    INFO_NET_GET_IP     = SIM800_NET_GET_IP,
    INFO_HTTP_RQST_GET  = SIM800_HTTP_RQST_GET,
    INFO_HTTP_RQST_POST = SIM800_HTTP_RQST_POST,
    INFO_HTTP_DATA_READ = SIM800_HTTP_DATA_READ,
    INFO_HTTP_DATA_SEND = SIM800_HTTP_DATA_SEND,
            
}info_rsp;

volatile sim800_oper_status_t g_mod_current_state;

//static sim800_conn_status_t g_gprs_current_state;
//
volatile network_status_t g_net_state;

static sim800_operator_t g_sim_oper;


void check_network ()
{
    
}

void check_gprs ()
{
}

void set_oper_specific_data ()
{
    memset (cmd_set_brr_apn, 0x00, sizeof(cmd_set_brr_apn));
    memset (cmd_tsk_start, 0x00, sizeof(cmd_tsk_start));
    
    hal_nop_delay_ms (100);
    
    static char l_ip_cntxt_head[] = {'A','T','+','C','G','D','C','O','N','T','=','1',',',
    '\"','I','P','\"',',','\"','\0'};    
    static char l_ip_cntxt_tail[] = {'\"','\r','\n','\0'};

    static char l_tsk_head[] = {'A','T','+','C','S','T','T','=','\"','\0'};
    static char l_tsk_tail[] = {'\"',',','\"','\"',',','\"','\"','\r','\n','\0'};

    static char l_set_brr_head[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','A','P','N','\"',',','\"','\0'};
    static char l_set_brr_tail[] = {'\"','\r','\n','\0'};

    strcpy (cmd_tsk_start, l_tsk_head);
    strcpy (cmd_set_brr_apn, l_set_brr_head);
    strcpy (cmd_ip_cntxt, l_ip_cntxt_head);
    
    switch (g_sim_oper)
    {
        case SIM800_AIRTEL :
        {
            strcat (cmd_tsk_start, "airtelgprs.com");
            strcat (cmd_set_brr_apn, "airtelgprs.com");
            strcat (cmd_ip_cntxt, "airtelgprs.com");
            break;
        }
        case SIM800_BSNL : 
        {
            strcat (cmd_tsk_start, "bsnlnet");
            strcat (cmd_set_brr_apn, "bsnlnet");
            strcat (cmd_ip_cntxt, "bsnlnet");
            break;
        }
        case SIM800_IDEA :
        {
            strcat (cmd_tsk_start, "IMIS");
            strcat (cmd_set_brr_apn, "IMIS");
            strcat (cmd_ip_cntxt, "IMIS");
            break;
        }
        case SIM800_VODAFONE :
        {
            strcat (cmd_tsk_start, "www");
            strcat (cmd_set_brr_apn, "www");
            strcat (cmd_ip_cntxt, "www");
            break;
        }
    }
    
    strcat (cmd_set_brr_apn, l_set_brr_tail);
    strcat (cmd_tsk_start, l_tsk_tail);
    strcat (cmd_ip_cntxt, l_ip_cntxt_tail);    
    
}

void critical_fail_handler (uint32_t cmd_id)
{
//TODO stop sim800_oper module if critical command fails after n reruns
//    if(g_rerun_cnt)
//    {
//    }
//    else
//    {
//        switch ((critical_cmd) cmd_id)
//        {
//            case CRITIC_MOD_INIT : 
//            {
//                break;
//            }
//        }
//    }
    AT_proc_repeat_last_cmd ();   
}


void reset_cmd (at_proc_cmd_t * cmd)
{
    memset (cmd, 0, sizeof(at_proc_cmd_t));
}

void send_data ()
{
    AT_proc_send_cmd_no_rsp ((uint8_t *)data_http_post, http_pdata_len, 5000);
}

void command_processed_successfully (uint32_t cmd_id, uint32_t rsp_id)
{
    log_printf("%s\n",__func__);
    
}

void command_unknown_response (uint32_t cmd_id, at_uart_data_t * u_data1, uint32_t len)
{
    log_printf("%s\n",__func__);
    
    char l_str[255];
    for(uint32_t cnt = 0; cnt < len; cnt++)
    {
        memset (l_str, 0, sizeof(l_str));
        memcpy (l_str, u_data1[cnt].ptr, u_data1[cnt].len);
        log_printf ("%s\n", l_str);
    }
    
    switch ((info_rsp)cmd_id)
    {
        case INFO_NET_BCHK : 
        {
            break;
        }
        
        case INFO_NET_GET_IP :
        {
            break;
        }
        
        case INFO_HTTP_DATA_SEND :
        {
            send_data ();
            break;
        }
        
        case INFO_HTTP_DATA_READ : 
        {
            
            break;
        }
        
        case INFO_HTTP_RQST_GET : 
        {
            break;
        }
        
        case INFO_HTTP_RQST_POST : 
        {
            break;
        }
        
    }
    
}

void command_process_failure (uint32_t cmd_id, uint8_t was_critical, uint8_t was_timeout,uint32_t error_id)
{
    log_printf("%s\n",__func__);
    
    if (was_critical)
    {
        critical_fail_handler (cmd_id);
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
    g_sim_oper = oper;
    set_oper_specific_data ();
    CBUF_Init(ATbuff);
    AT_proc_init (&at_init);
    //Add Init Seq to command buffer
    at_proc_cmd_t l_at_cmd;

    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_INIT;
    l_at_cmd.cmd.ptr = (char *)cmd_init;
    l_at_cmd.cmd.len = sizeof(cmd_init);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = (char *)cmd_fac_reset;
    l_at_cmd.cmd.len = sizeof(cmd_fac_reset);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.cmd_id = SIM800_MOD_FCT_RST;
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_TIME_FMT;
    l_at_cmd.cmd.ptr = (char *)cmd_time_fmt;
    l_at_cmd.cmd.len = sizeof(cmd_time_fmt);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_SAVE_NVM;
    l_at_cmd.cmd.ptr = (char *)cmd_save_nvm;
    l_at_cmd.cmd.len = sizeof(cmd_save_nvm);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_EN_FUNC;
    l_at_cmd.cmd.ptr = (char *)cmd_en_full;
    l_at_cmd.cmd.len = sizeof(cmd_en_full);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);

    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_CHK_SIM;
    l_at_cmd.cmd.ptr = (char *)cmd_chk_sim;
    l_at_cmd.cmd.len = sizeof(cmd_chk_sim);
    l_at_cmd.resp[0].ptr = (char *)rsp_sim_sts1;
    l_at_cmd.resp[0].len = sizeof(rsp_sim_sts1);
    l_at_cmd.err[0].ptr = (char *)err_sim_sts1;
    l_at_cmd.err[0].len = sizeof(err_sim_sts1);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
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
    at_proc_cmd_t l_at_cmd;
    //Network
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_CHK;
    l_at_cmd.cmd.ptr = (char *)cmd_nw_reg;
    l_at_cmd.cmd.len = sizeof(cmd_nw_reg);
    l_at_cmd.resp[0].ptr = (char *)rsp1l2_gprs_reg;
    l_at_cmd.resp[0].len = sizeof(rsp1l2_gprs_reg);
    l_at_cmd.resp[1].ptr = (char *)rsp2l2_gprs_reg;
    l_at_cmd.resp[1].len = sizeof(rsp2l2_gprs_reg);
    l_at_cmd.err[0].ptr = (char *)err1l2_gprs_reg;
    l_at_cmd.err[0].len = sizeof(err1l2_gprs_reg);
    l_at_cmd.err[1].ptr = (char *)err2l2_gprs_reg;
    l_at_cmd.err[1].len = sizeof(err2l2_gprs_reg);
    l_at_cmd.err[2].ptr = (char *)err3l2_gprs_reg;
    l_at_cmd.err[2].len = sizeof(err3l2_gprs_reg);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 15000;
    CBUF_Push(ATbuff, l_at_cmd);
    
    //Add GPRS
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BSET_CTYPE;
    l_at_cmd.cmd.ptr = (char *)cmd_set_brr_ctype;
    l_at_cmd.cmd.len = sizeof(cmd_set_brr_ctype);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BSET_APN;
    l_at_cmd.cmd.ptr = (char *)cmd_set_brr_apn;
    l_at_cmd.cmd.len = sizeof(cmd_set_brr_apn);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BSET_USR;
    l_at_cmd.cmd.ptr = (char *)cmd_set_brr_usr;
    l_at_cmd.cmd.len = sizeof(cmd_set_brr_usr);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BSET_PWD;
    l_at_cmd.cmd.ptr = (char *)cmd_set_brr_pwd;
    l_at_cmd.cmd.len = sizeof(cmd_set_brr_pwd);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_PDP_CNTX;
    l_at_cmd.cmd.ptr = (char *)cmd_ip_cntxt;
    l_at_cmd.cmd.len = sizeof(cmd_ip_cntxt);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_ACTV_CID;
    l_at_cmd.cmd.ptr = (char *)cmd_actv_cid;
    l_at_cmd.cmd.len = sizeof(cmd_actv_cid);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BEN;
    l_at_cmd.cmd.ptr = (char *)cmd_actv_brr;
    l_at_cmd.cmd.len = sizeof(cmd_actv_brr);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_BCHK;
    l_at_cmd.cmd.ptr = (char *)cmd_chk_brr;
    l_at_cmd.cmd.len = sizeof(cmd_chk_brr);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_EN_MUX;
    l_at_cmd.cmd.ptr = (char *)cmd_en_mux;
    l_at_cmd.cmd.len = sizeof(cmd_en_mux);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_SEL_DTX;
    l_at_cmd.cmd.ptr = (char *)cmd_sel_dtx_mode;
    l_at_cmd.cmd.len = sizeof(cmd_sel_dtx_mode);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_TSK_START;
    l_at_cmd.cmd.ptr = (char *)cmd_tsk_start;
    l_at_cmd.cmd.len = sizeof(cmd_tsk_start);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_EN_GPRS;
    l_at_cmd.cmd.ptr = (char *)cmd_con_gprs;
    l_at_cmd.cmd.len = sizeof(cmd_con_gprs);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
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
        return;
    }
    return;
}

void sim800_oper_add_ticks (uint32_t ticks)
{
    static uint32_t l_current_ticks = 0;
    
    //pass on these ticks to AT_proc
    if(AT_proc_is_busy ())
    {
        l_current_ticks = 0;
        AT_proc_add_ticks (ticks);
    }
    else
    {
        l_current_ticks += ticks;
    }
    if (l_current_ticks >= NET_CHK_FREQ)
    {
        l_current_ticks = 0;
        check_network ();
        check_gprs ();
    }
}

uint32_t sim800_oper_conns (sim800_server_conn_t * conn_params)
{
    char l_http_head[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
        'U','R','L','\"',',','\"','\0'};
    char l_http_tail[] = {'\"','\r','\n','\0'};
    strcpy (cmd_http_url, l_http_head);
    strcat (cmd_http_url, conn_params->server_ptr);
    //ToDo calc max resource len
    if ((conn_params->resource_ptr) && (conn_params->resource_len < MAX_LEN_CMD))
    {
        strcat (cmd_http_url, "/");
        strcat (cmd_http_url, conn_params->resource_ptr);
    }
    if ((conn_params->port_ptr) && (conn_params->port_len < MAX_LEN_CMD))
    {
        strcat (cmd_http_url, ":");
        strcat (cmd_http_url, conn_params->port_ptr );
    }
    
    strcat (cmd_http_url, l_http_tail);
    
    log_printf ("HTTP Param : %s\n", cmd_http_url);
    return 0;
}


void reverse(char *s)
{
    uint32_t length, c;
    char *begin, *end, temp;

    length = strlen (s);
    begin  = s;
    end    = s;

    for (c = 0; c < length - 1; c++)
    {
        end++;
    }

    for (c = 0; c < length/2; c++)
    {        
        temp   = *end;
        *end   = *begin;
        *begin = temp;

        begin++;
        end--;
    }
}
 
void assign_data_len (uint32_t len)
{
    char l_set_pdata_head[] = {'A','T','+','H','T','T','P','D','A','T','A','=','\0'};
    char l_set_pdata_tail[] = {',','7','0','0','0','\r','\n','\0'};
    char l_int_str[12];
    memset (l_int_str, 0, sizeof(l_int_str));
    uint32_t i = 0;
    strcpy (cmd_http_set_pdata, l_set_pdata_head);
    if (len == 0)
    {
        l_int_str[i++] = '0';
        l_int_str[i] = '\0';
    }
    else
    {
        while (len != 0) 
        { 
            int rem = len % 10; 
            l_int_str[i++] = rem + '0'; 
            len = len/10; 
        }
        reverse (l_int_str);
        
    }
    strcat (cmd_http_set_pdata, l_int_str);
    strcat (cmd_http_set_pdata, l_set_pdata_tail);
}

void sim800_oper_http_req (sim800_http_req_t * http_req)
{
    
    
    at_proc_cmd_t l_at_cmd;
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = (char *)cmd_http_init;
    l_at_cmd.cmd.len = sizeof(cmd_http_init);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = (char *)cmd_http_cid;
    l_at_cmd.cmd.len = sizeof(cmd_http_cid);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = (char *)cmd_http_url;
    l_at_cmd.cmd.len = sizeof(cmd_http_url);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 0;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
    
    if(http_req->req_type == SIM800_HTTP_GET)
    {

        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd.ptr = (char *)cmd_http_get;
        l_at_cmd.cmd.len = sizeof(cmd_http_get);
        l_at_cmd.resp[0].ptr = (char *)rsp_http_set_pdata;
        l_at_cmd.resp[0].len = sizeof(rsp_http_set_pdata);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = 0;
        l_at_cmd.is_response_variable = 1;
        l_at_cmd.timeout = 7000;
        CBUF_Push(ATbuff, l_at_cmd);
    }
    else
    {
        assign_data_len (http_req->len);
        
        log_printf ("Set Data req : %s\n", cmd_http_set_pdata);
    
        memcpy (data_http_post, http_req->payload_ptr, http_req->len);
        
        http_pdata_len = http_req->len;
        
        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd_id = 0x3333;
        l_at_cmd.cmd.ptr = (char *)cmd_http_set_pdata;
        l_at_cmd.cmd.len = sizeof(cmd_http_set_pdata);
        l_at_cmd.resp[0].ptr = rsp_std_OK;
        l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = 0;
        l_at_cmd.is_response_variable = 1;
        l_at_cmd.timeout = 2000;
        CBUF_Push(ATbuff, l_at_cmd);

        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd.ptr = (char *)cmd_http_post;
        l_at_cmd.cmd.len = sizeof(cmd_http_post);
        l_at_cmd.resp[0].ptr = rsp_std_OK;
        l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = 0;
        l_at_cmd.is_response_variable = 1;
        l_at_cmd.timeout = 5000;
        CBUF_Push(ATbuff, l_at_cmd);

    }
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd.ptr = (char *)cmd_http_read;
    l_at_cmd.cmd.len = sizeof(cmd_http_read);
    l_at_cmd.is_critical = 0;
    l_at_cmd.is_response_variable = 1;
    l_at_cmd.timeout = 2500;
    CBUF_Push(ATbuff, l_at_cmd);
        
//    reset_cmd (&l_at_cmd);
//    l_at_cmd.cmd.ptr = (char *)cmd_http_term;
//    l_at_cmd.cmd.len = sizeof(cmd_http_term);
//    l_at_cmd.resp[0].ptr = rsp_std_OK;
//    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
//    l_at_cmd.err[0].ptr = rsp_std_ERR;
//    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
//    l_at_cmd.is_critical = 0;
//    l_at_cmd.is_response_variable = 0;
//    l_at_cmd.timeout = 2500;
//    CBUF_Push(ATbuff, l_at_cmd);
        
}

sim800_conn_status_t sim800_oper_get_gprs_status ()
{
    return SIM800_CONNECTED;
}

sim800_conn_status_t sim800_oper_get_server_status (uint32_t server_id)
{
    return SIM800_CONNECTED;
}


