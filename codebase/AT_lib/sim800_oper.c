/*
 *  sim800_oper.c : Module to access SIM800 HW module with AT commands
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


#include <stdlib.h>

#include "sim800_oper.h"
#include "AT_proc.h"
#include "sim800_cmd_id.h"
#include "log.h"


#include "stdbool.h"
#include "string.h"


/** @brief Size of the ring buffer for the message list
 *  @note Should be a power of 2 */
#define ATbuff_SIZE (64)

#include "CBUF.h"
#include "hal_nop_delay.h"
#include "ms_timer.h"
/** Check if MSG_SIZE is power of 2 */
#if (!(!(ATbuff_SIZE & (ATbuff_SIZE-1)) && ATbuff_SIZE))
#error ATbuff_SIZE must be a power of 2
#endif

/** Maximum command length in bytes */
#define MAX_LEN_CMD (128)
/** Maximum Response length in bytes */
#define MAX_LEN_RSP (512)

/** Macro to convert seconds in milliseconds */
#define S_to_MS(x)          (x * 1000)
/** Network check frequency in S */
#define NET_CHK_FREQ_S      (20)
/** Network check frequency in milliseconds */
#define NET_CHK_FREQ_MS     S_to_MS(NET_CHK_FREQ_S)
/** Network check frequency in ticks */
#define NET_CHK_FREQ_TICKS  (MS_TIMER_TICKS_MS(NET_CHK_FREQ_MS))



volatile static struct
{
    uint32_t m_getIdx;
    uint32_t m_putIdx;
    at_proc_cmd_t m_entry[ATbuff_SIZE];
} ATbuff;

/** AT Commands */

/** Standard Response and Error */
/** Standard Response */
static char rsp_std_OK[] = {'O','K','\r','\n'};
/** Standard Error */
static char rsp_std_ERR[] = {'E','R','R','O','R','\r','\n'};

/** Module control commands */
/** Command to initialize and check AT module */
const char cmd_init[] = {'A','T','\r','\n'};
/** Command to do factory reset */
const char cmd_fac_reset[] = {'A','T','&','F','Z','\r','\n'};
/** Command to turn off echo from SIM800 module */
static char cmd_echo_off[] = {'A','T','E','0','\r','\n'};
/** Command to set default time format */
const char cmd_time_fmt[] = {'A','T','+','C','L','T','S','=','1','\r','\n'};
/** Command to save configuration in Non Volatile memory present in SIM800 module */
const char cmd_save_nvm[] = {'A','T','&','W','\r','\n'};
/** Command to disable radio functionality */
const char cmd_dis_full[] = {'A','T','+','C','F','U','N','=','0','\r','\n'};
/** Command to Enable radio functionality */
const char cmd_en_full[] = {'A','T','+','C','F','U','N','=','1','\r','\n'}; //Res : std reply

/** Commmand to check sim status */
const char cmd_chk_sim[] = {'A','T','+','C','P','I','N','?','\r','\n'};
/** Expected response if SIM is present and ready to be used */
const char rsp_sim_sts1[] = {'+','C','P','I','N',':',' ','R','E','A','D','Y','\r','\n'};
/** Expected Error message if SIM is not present or not ready */
const char err_sim_sts1[] = {'+','C','P','I','N',':',' ','E','R','R','O','R','\r','\n'};
/** Command to switch-off SIM800 module */
const char cmd_power_off[] = {'A','T','+','C','P','O','W','D','=','1','\r','\n'};


/** Network Commands */
/**
 * Check for network
 * AT+CREG? -- 
 */
/** Command to check network registration */
const char cmd_nw_reg[] = {'A','T','+','C','R','E','G','?','\r','\n'};
/** Expected response if network registration is done at home region */
const char rsp1l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','1','\r','\n'};
/**Expected response if network registration is done in roaming network */
const char rsp2l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','5','\r','\n'};
/** Expected error when network registration is not happening */
const char err1l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','0','\r','\n'};
/** Expected error when module is searching for network */
const char err2l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','2','\r','\n'};
/** Expected error when unknown error occurs at module's end */
const char err3l2_gprs_reg[] = {'+','C','R','E','G',':',' ','0',',','4','\r','\n'};

/** Command to check GPRS connectivity */
const char cmd_chk_gprs[] = {'A','T','+','C','G','A','T','T','?','\r','\n'};
/** Expected response if GPRS is connected */
const char rsp_chk_gprs[] = {'+','C','G','A','T','T',':',' ','1','\r','\n'};
/** Expected error if GPRS is disconnected */
const char err_chk_gprs[] = {'+','C','G','A','T','T',':',' ','0','\r','\n'};

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
/** Command to set Bearer connection type */
const char cmd_set_brr_ctype[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','C','o','n','t','y','p','e','\"',',','\"','G','P','R','S','\"','\r','\n'};
/** Command to set Bearer APN */
static char cmd_set_brr_apn[MAX_LEN_CMD];
/** Command to set Bearer USER */
const char cmd_set_brr_usr[]= {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','U','S','E','R','\"',',','\"','\"','\r','\n'};
/** Command to set Bearer Password */
const char cmd_set_brr_pwd[] = {'A','T','+','S','A','P','B','R','=','3',',','1',',',
    '\"','P','W','D','\"',',','\"','\"','\r','\n'};

/** Command to set PDP Context */
static char cmd_ip_cntxt[MAX_LEN_CMD];
/** Command to Activate CID */
const char cmd_actv_cid[] = {'A','T','+','C','G','A','C','T','=',
'1',',','1','\r','\n'};
/** Command to activate Bearer */
const char cmd_actv_brr[] = {'A','T','+','S','A','P','B','R','=','1',',','1','\r','\n'};
/** Command to check Bearer status */
const char cmd_chk_brr[] = {'A','T','+','S','A','P','B','R','=','2',',','1','\r','\n'};
/** Command to enable Context multiplexing */
const char cmd_en_mux[] =  {'A','T','+','C','I','P','M','U','X','=','1','\r','\n'};
/** Command to set Data transmission mode */
const char cmd_sel_dtx_mode[] = {'A','T','+','C','I','P','Q','S','E','N','D','=','1','\r','\n'};
/** Command to set Data reception mode (untested) */
const char cmd_get_net_data[] = {'A','T','+','C','I','P','R','X','G','E','T','=','1','\r','\n'};
/** Command to start task with APN, USER and Password */
static char cmd_tsk_start[MAX_LEN_CMD];
/** Command to connect to GPRS */
const char cmd_con_gprs[] = {'A','T','+','C','I','I','C','R','\r','\n'};
/** Alternative command to connect to GPRS */
const char cmd_gprs_attach[] = {'A','T','+','G','A','T','T','=','1','\r','\n'};
/** Command to get IP address of SIM800 module */
const char cmd_get_ip[] = {'A','T','+','C','I','F','S','R','\r','\n'};
/** Command to set DNS server (untested)*/
const char cmd_set_dns[] = {'A','T','+','C','D','N','S','C','F','G','=',
'\"','8','.','8','.','8','.','8','\"',',','\"','8','.','8','.','4','.','4','\"','\r','\n'};

/** HTTP commands */
/** Command to initialize HTTP functionality */
const char cmd_http_init[] = {'A','T','+','H','T','T','P','I','N','I','T','\r','\n'};
/** Command to set HTTP parameter : CID */
const char cmd_http_cid[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
'C','I','D','\"',',','1','\r','\n'};
/** Command to set HTTP parameter : Content type */ //TODO : Make content type variable
const char cmd_http_type[] = {'A','T','+','H','T','T','P','P','A','R','A','=','\"',
'C','O','N','T','E','N','T','\"',',','\"','t','e','x','t','/','p','l','a','i','n','\"','\r','\n'};
/** Command to set HTTP parameter : URL */
static char cmd_http_url[MAX_LEN_CMD];
/** Command to make GET request */
const char cmd_http_get[] = {'A','T','+','H','T','T','P','A','C','T','I','O','N',
'=','0','\r','\n'};
/** Command to make POST request */
const char cmd_http_post[] = {'A','T','+','H','T','T','P','A','C','T','I','O','N',
'=','1','\r','\n'};
/** Command to set data enable which is to be sent */
static char cmd_http_set_pdata[MAX_LEN_CMD];
/** Expected response for set data enable command */
const char rsp_http_set_pdata[] = {'D','O','W','N','L','O','A','D','\n'};
/** Command to read data received in response */
const char cmd_http_read[] = {'A','T','+','H','T','T','P','R','E','A','D','\r','\n'};
/** Command to terminate HTTP session */
const char cmd_http_term[] = {'A','T','+','H','T','T','P','T','E','R','M','\r','\n'};

/** Enums */
/** List of possible states when critical command is being executed */
typedef enum
{
    CCMD_DONE,
    CCMD_RUNNING,
    CCMD_FAIL,
}crit_cmd_status_t;

/** List of all the critical commands */
typedef enum
{
    CRITIC_MOD_INIT     = SIM800_MOD_INIT,
    CRITIC_MOD_CHK_SIM  = SIM800_MOD_CHK_SIM,
    CRITIC_NET_CHK      = SIM800_NET_CRT_CHK,
    CRITIC_NET_EN_GPRS  = SIM800_NET_EN_GPRS,            
}critical_cmd;

/** List of number of reruns for each critical command before module goes to HALT state */
enum critical_cmd_reruns
{
    MOD_INIT_RERUN      = 10,
    MOD_CHK_SIM_RERUN   = 5,
    NET_CHK             = 30,
    NET_EN_GPRS         = 10,
};

/** List of all commands where module is not expecting any strictly defined responses */
typedef enum
{
    INFO_NET_BCHK       = SIM800_NET_BCHK,
    INFO_NET_GET_IP     = SIM800_NET_GET_IP,
    INFO_HTTP_RQST_GET  = SIM800_HTTP_RQST_GET,
    INFO_HTTP_RQST_POST = SIM800_HTTP_RQST_POST,
    INFO_HTTP_DATA_READ = SIM800_HTTP_DATA_READ,
    INFO_HTTP_DATA_SEND = SIM800_HTTP_DATA_SEND,
            
}info_rsp;

/** Global variables */
/** Array to store Data which is to be sent with request */
static char data_http_post[MAX_LEN_CMD];
/** Variable to store Data length of HTTP data */
static uint32_t http_pdata_len = 0;
/** Variable to store current state of SIM800 module */
volatile sim800_oper_status_t g_mod_current_state;
/** Variable to store current GPRS status */
static sim800_conn_status_t g_gprs_current_state = SIM800_DISCONNECTED;
/** Variable to store status of critical command */
static crit_cmd_status_t g_ccmd_status;

/** Variable to store number of reruns for current critical command */
static uint32_t g_rerun_cnt = 0;
/** Variable to store SIM operator */
static sim800_operator_t g_sim_oper;
/** Variable to store the auto connect flag  */
static bool g_is_autoconn_en = true;
/** Variable to store status code for last HTTP operation */
static uint32_t g_http_status_code = 0;
/** Function pointer to the callback function which is to be called when SIM800 state is changed */
void (* p_oper_state_changed) (sim800_oper_status_t new_sts);
/** Function pointer to the callback function which is to be called when GPRS state is changed */
void (* p_gprs_state_changed) (sim800_conn_status_t new_sts);
/** Function pointer to the callback function which is to be called when HTTP response is received */
void (* p_http_response) (uint32_t status_code);

/** Local support functions */

/**
 * @brief Function to update Module's state
 * @param new_state New state of module
 */
void update_mod_status (sim800_oper_status_t new_state)
{
    if (g_mod_current_state != new_state)
    {
        g_mod_current_state = new_state;
        if (p_oper_state_changed)
        {
            p_oper_state_changed (new_state);
        }
    }
}

/**
 * @brief Function to update GPRS state
 * @param new_state New state of GPRS
 */
void update_gprs_status (sim800_conn_status_t new_state)
{
    if (g_gprs_current_state != new_state)
    {
        g_gprs_current_state = new_state;
        if (p_gprs_state_changed)
        {
            p_gprs_state_changed (new_state);
        }
    }
}

/**
 * @brief Function to update status code of last HTTP request
 * @param code New code.
 */
void update_htttp_status_code (uint32_t code)
{
    if (code)
    {
        g_http_status_code = code;
        if (p_http_response)
        {
            p_http_response (code);
        }
    }
}

/**
 * @brief Function to extract status code from the response received from AT module
 * @param str Response string received from AT process module
 * @return Status code if 
 */
uint32_t get_status_code (char * str)
{
    log_printf ("%s : %s\n", __func__, str);
    //Expected response +HTTPACTION: x,yyy,zzzz
    const char l_hact_rsp[] = {'+','H','T','T','P','A','C','T','I','O','N',':',' '};
    if(memcmp (str, l_hact_rsp, sizeof(l_hact_rsp)))
    {
        return 0;
    }
    else
    {
        uint8_t rcode_s[4];
        memcpy (rcode_s, &str[sizeof(l_hact_rsp) + 2], 3); //+2 is to handle "x," part
        rcode_s[3] = '\0';
        return (uint32_t)  atoi ((char*)rcode_s);
    }
}

/**
 * @brief Function to reset an AT command 
 * @param cmd Pointer to the command which is to reset
 */
void reset_cmd (at_proc_cmd_t * cmd)
{
    memset (cmd, 0, sizeof(at_proc_cmd_t));
}

/**
 * @brief Function to push command in circular buffer 
 * @param cmd Pointer to command which is to be pushed
 */
void push_cmd (at_proc_cmd_t cmd)
{
    if (CBUF_Len(ATbuff) < ATbuff_SIZE)
    {
        CBUF_Push(ATbuff, cmd);
    }
    else
    {
        update_mod_status (SIM800_OVERLOAD);
        //TODO : Handle Overload condition
    }
}

/**
 * @brief Function to reverse the given string 
 * @param s String which is to be reversed
 */
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

/**
 * @brief Function to assign data length before HTTP request is made 
 * @param len Length of data which is to be sent with HTTP request
 */
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

/**
 * @brief Function to set operator specific data (APN) for different commands 
 */
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

/**
 * @brief Function to reconnect to GSM+GPRS network 
 */
void reconnect ()
{
    at_proc_cmd_t l_at_cmd;
    log_printf ("Trying to reconnect\n");

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
    push_cmd(l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_DIS_FUNC;
    l_at_cmd.cmd.ptr = (char *)cmd_dis_full;
    l_at_cmd.cmd.len = sizeof(cmd_dis_full);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);

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
    push_cmd(l_at_cmd);

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
    l_at_cmd.timeout = 6000;
    push_cmd(l_at_cmd);

    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_CRT_CHK;
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
    push_cmd(l_at_cmd);

    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
}

/**
 * @brief Function to check GPRS connectivity 
 */
void check_gprs ()
{
    at_proc_cmd_t l_at_cmd;
    //Network
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_CHK_GPRS;
    l_at_cmd.cmd.ptr = (char *)cmd_chk_gprs;
    l_at_cmd.cmd.len = sizeof(cmd_chk_gprs);
    l_at_cmd.resp[0].ptr = (char *)rsp_chk_gprs;
    l_at_cmd.resp[0].len = sizeof(rsp_chk_gprs);
    l_at_cmd.err[0].ptr = (char *)err_chk_gprs;
    l_at_cmd.err[0].len = sizeof(err_chk_gprs);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 12000;
    push_cmd(l_at_cmd);
}

/**
 * @brief Function to check GSM network 
 */
void check_network ()
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
    push_cmd(l_at_cmd);
}

/**
 * @brief Function to set number of reruns for critical command 
 * @param cmd_id ID according to which rerun number is decided
 */
void assign_rerun_count (critical_cmd cmd_id)
{
    switch (cmd_id)
    {
        case CRITIC_MOD_INIT : 
        {
            g_rerun_cnt = MOD_INIT_RERUN;
            break;
        }
        case CRITIC_MOD_CHK_SIM :
        {
            g_rerun_cnt = MOD_CHK_SIM_RERUN;
            break;
        }
        case CRITIC_NET_CHK :
        {
            g_rerun_cnt = NET_CHK;
            break;
        }
        case CRITIC_NET_EN_GPRS :
        {
            g_rerun_cnt = NET_EN_GPRS;
            break;
        }
    }
}

/**
 * @brief Function to handle critical command failure 
 * @param cmd_id Command ID of command which failed
 */
void critical_fail_handler (uint32_t cmd_id)
{
    switch (g_ccmd_status)
    {
        case CCMD_DONE : 
        {
            assign_rerun_count ((critical_cmd)cmd_id);
            g_ccmd_status = CCMD_RUNNING;
            AT_proc_repeat_last_cmd ();   
            break;
        }
        case CCMD_RUNNING :
        {
            if (g_rerun_cnt)
            {
                g_rerun_cnt--;
            }
            else
            {
                g_ccmd_status = CCMD_FAIL;
            }
            AT_proc_repeat_last_cmd ();   
            break;
        }
        case CCMD_FAIL :
        {
            if (cmd_id == CRITIC_MOD_INIT)
            {
                update_mod_status (SIM800_NOT_FOUND);
            }
            else
            {
                update_mod_status (SIM800_HALT);
            }
            
            break;
        }
    }
}

/**
 * @brief Function to handle normal command failure 
 * @param cmd_id Command ID of command which failed
 */
void normal_fail_handler (uint32_t cmd_id)
{
    switch (cmd_id)
    {
        case SIM800_NET_CHK : 
        {
            update_gprs_status (SIM800_DISCONNECTED);
            log_printf("GSM failed\n");
            if (g_is_autoconn_en)
            {
                reconnect ();
            }            
            break;
        }
        case SIM800_NET_CHK_GPRS : 
        {
            update_gprs_status (SIM800_DISCONNECTED);
            log_printf("GPRS failed\n");
            if (g_is_autoconn_en)
            {
                reconnect ();
            }
            break;
        }
        default :
            break;
    }
}

/**
 * @brief Function to stream data while making HTTP request 
 */
void stream_req_data ()
{
    log_printf("Sending Data...!\n");
    AT_proc_send_cmd_no_rsp ((uint8_t *)data_http_post, http_pdata_len, 5000);
}

/**
 * @brief Callback function which will be called if command is executed successfully
 * @param cmd_id Command which was being executed
 * @param rsp_id Response id of response that module got
 */
void command_processed_successfully (uint32_t cmd_id, uint32_t rsp_id)
{
    log_printf("%s\n",__func__);
    switch (cmd_id)
    {
        case SIM800_NET_CHK : 
        {
            log_printf ("GSM is working fine\n");
            check_gprs ();
            break;
        }
        case SIM800_NET_CHK_GPRS : 
        {
            update_gprs_status (SIM800_CONNECTED);
            log_printf ("GPRS is working fine\n");
            break;
        }
        case SIM800_NET_EN_GPRS : 
        {
            update_gprs_status (SIM800_CONNECTED);
            log_printf ("GPRS Connected...\n");
        }
        
        default :
            break;
    }
}
/**
 * @brief Callback function which will be called when there is no well defined response available for command 
 * @param cmd_id Command which was being executed
 * @param u_data1 Structure pointer of type at_uart_data_t @ref at_uart_data_t
 * @param len Length of array of type at_uart_data_t
 */
void command_unknown_response (uint32_t cmd_id, at_uart_data_t * u_data1, uint32_t len)
{
    log_printf("%s\n",__func__);
    
    char l_str[255];
    for(uint32_t cnt = 0; cnt < len; cnt++)
    {
        memset (l_str, 0, sizeof(l_str));
        memcpy (l_str, u_data1[cnt].ptr, u_data1[cnt].len);
        log_printf ("%s\n", l_str);    
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
                stream_req_data ();
                break;
            }

            case INFO_HTTP_DATA_READ : 
            {
                break;
            }

            case INFO_HTTP_RQST_GET : 
            {
//                update_htttp_status_code (get_status_code (l_str));
                break;
            }

            case INFO_HTTP_RQST_POST : 
            {
                break;
            }

        }
        update_htttp_status_code (get_status_code (l_str));
    }
    
}

/**
 * @brief Callback function which will be called when command executions fails  
 * @param cmd_id Command ID of command which was being executed
 * @param was_critical Flag which stores if command was critical 
 * @param was_timeout Flag which stores if command failed because of Timeout
 * @param error_id Error ID of error that caused command failure
 */
void command_process_failure (uint32_t cmd_id, uint8_t was_critical, uint8_t was_timeout,uint32_t error_id)
{
    log_printf("%s\n",__func__);
    
    if (was_critical)
    {
        critical_fail_handler (cmd_id);
    }
    else
    {
        normal_fail_handler (cmd_id);
    }
}


/** Refer sim800_oper.h for documentation */
void sim800_oper_init (sim800_init_t * init)
{
    AT_proc_init_t at_init = 
    {
        .cmd_successful = command_processed_successfully,
        .cmd_successful_data = command_unknown_response,
        .cmd_failed = command_process_failure,
    };
    p_http_response = init->sim800_http_response;
    p_oper_state_changed = init->sim800_oper_state_changed;
    p_gprs_state_changed = init->sim800_gprs_state_changed;
    
    update_mod_status (SIM800_IDLE);
    update_gprs_status (SIM800_DISCONNECTED);
    g_sim_oper = init->operator;
    g_is_autoconn_en = init->autoconn_enable;
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
    push_cmd(l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_FCT_RST;
    l_at_cmd.cmd.ptr = (char *)cmd_fac_reset;
    l_at_cmd.cmd.len = sizeof(cmd_fac_reset);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_MOD_ECHO_OFF;
    l_at_cmd.cmd.ptr = (char *)cmd_echo_off;
    l_at_cmd.cmd.len = sizeof(cmd_echo_off);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);

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
    l_at_cmd.timeout = 6000;
    push_cmd(l_at_cmd);

    
}


//void sim800_oper_enable_sms (void)
//{
//}

void sim800_oper_enable_gprs (void)
{
    at_proc_cmd_t l_at_cmd;
    //Network
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_NET_CRT_CHK;
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    push_cmd(l_at_cmd);
    
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
    
    return 0;
}

void sim800_oper_http_req (sim800_http_req_t * http_req)
{

    at_proc_cmd_t l_at_cmd;
    
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_HTTP_INIT;
    l_at_cmd.cmd.ptr = (char *)cmd_http_init;
    l_at_cmd.cmd.len = sizeof(cmd_http_init);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_HTTP_PARA_CID;
    l_at_cmd.cmd.ptr = (char *)cmd_http_cid;
    l_at_cmd.cmd.len = sizeof(cmd_http_cid);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_HTTP_PARA_URL;
    l_at_cmd.cmd.ptr = (char *)cmd_http_url;
    l_at_cmd.cmd.len = sizeof(cmd_http_url);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);

    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_HTTP_PARA_CTYPE;
    l_at_cmd.cmd.ptr = (char *)cmd_http_type;
    l_at_cmd.cmd.len = sizeof(cmd_http_type);
    l_at_cmd.resp[0].ptr = rsp_std_OK;
    l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
    l_at_cmd.err[0].ptr = rsp_std_ERR;
    l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
    
    if(http_req->req_type == SIM800_HTTP_GET)
    {

        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd_id = SIM800_HTTP_RQST_GET;
        l_at_cmd.cmd.ptr = (char *)cmd_http_get;
        l_at_cmd.cmd.len = sizeof(cmd_http_get);
        l_at_cmd.resp[0].ptr = (char *)rsp_http_set_pdata;
        l_at_cmd.resp[0].len = sizeof(rsp_http_set_pdata);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
        l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
        l_at_cmd.timeout = 7000;
        push_cmd(l_at_cmd);
    }
    else
    {
        assign_data_len (http_req->len);
    
        memcpy (data_http_post, http_req->payload_ptr, http_req->len);
        
        http_pdata_len = http_req->len;
        
        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd_id = SIM800_HTTP_DATA_SEND;
        l_at_cmd.cmd.ptr = (char *)cmd_http_set_pdata;
        l_at_cmd.cmd.len = sizeof(cmd_http_set_pdata);
        l_at_cmd.resp[0].ptr = rsp_std_OK;
        l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
        l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
        l_at_cmd.timeout = 2000;
        push_cmd(l_at_cmd);

        reset_cmd (&l_at_cmd);
        l_at_cmd.cmd_id = SIM800_HTTP_POST;
        l_at_cmd.cmd.ptr = (char *)cmd_http_post;
        l_at_cmd.cmd.len = sizeof(cmd_http_post);
        l_at_cmd.resp[0].ptr = rsp_std_OK;
        l_at_cmd.resp[0].len = sizeof(rsp_std_OK);
        l_at_cmd.err[0].ptr = rsp_std_ERR;
        l_at_cmd.err[0].len = sizeof(rsp_std_ERR);
        l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
        l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
        l_at_cmd.timeout = 8000;
        push_cmd(l_at_cmd);

    }
        
    reset_cmd (&l_at_cmd);
    l_at_cmd.cmd_id = SIM800_HTTP_DATA_READ;
    l_at_cmd.cmd.ptr = (char *)cmd_http_read;
    l_at_cmd.cmd.len = sizeof(cmd_http_read);
    l_at_cmd.is_critical = IS_CMD_CRITICAL(l_at_cmd.cmd_id);
    l_at_cmd.is_response_variable = IS_RSP_VARIABLE(l_at_cmd.cmd_id);
    l_at_cmd.timeout = 2500;
    push_cmd(l_at_cmd);
        
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
//    push_cmd(l_at_cmd);
        
}

void sim800_oper_process ()
{
    if (g_mod_current_state == SIM800_IDLE)
    {
        at_proc_cmd_t l_cmd;
        if(AT_proc_is_busy ())
        {
            AT_proc_process ();
        }
        else if(CBUF_Len(ATbuff))
        {
            l_cmd = CBUF_Pop(ATbuff);
            AT_proc_send_cmd (&l_cmd);
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }
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
        if (l_current_ticks >= NET_CHK_FREQ_TICKS)
        {
            l_current_ticks = 0;
            check_network ();
        }
    }
}

sim800_conn_status_t sim800_oper_get_gprs_status ()
{
    return g_gprs_current_state;
}

sim800_oper_status_t sim800_oper_get_status ()
{
    return g_mod_current_state;
}
