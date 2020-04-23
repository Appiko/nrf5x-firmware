/*
 *  sim800_cmd_id.h : File to store Command IDs
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
 * @defgroup group_sim800 SIM800 Command IDs
 * @brief List of all the command IDs required for SIM800 operation
 *
 * @{
 */

#ifndef SIM800_CMD_ID_H
#define SIM800_CMD_ID_H

#ifdef __cplusplus
extern "C" {
#endif

/** Primary Base */
/** Base value for Module related commands */
#define SIM800_MOD_CMD_BASE (0x0000)
/** Base value for network related commands */
#define SIM800_NET_CMD_BASE (0x1000)
/** Base value for HTTP related commands */
#define SIM800_HTTP_CMD_BASE (0x2000)

/** Secondary Base */
/** Base value for commands which has definite responses, errors and isn't critical*/
#define SIM800_NORMAL_CMD_BASE (0x000)
/** Base value for commands which has definite responses, errors and is critical*/
#define SIM800_CRITICAL_CMD_BASE (0x100)
/** Base value for commands which has variable response */
#define SIM800_INFO_CMD_BASE (0x200)
  
/** Macro to check if command is critical */
#define IS_CMD_CRITICAL(x) ((x & SIM800_CRITICAL_CMD_BASE) >> 8)
/** Macro to check if variable response is expected */
#define IS_RSP_VARIABLE(x) ((x & SIM800_INFO_CMD_BASE) >> 9)

/** List of Command IDs for SIM800 module related commands */
enum SIM800_MOD_CMD
{
    SIM800_MOD_INIT     = (SIM800_MOD_CMD_BASE+SIM800_CRITICAL_CMD_BASE+1),///AT
    SIM800_MOD_FCT_RST  = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+2),///AT&FZ       
    SIM800_MOD_TIME_FMT = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+3),///AT+CLTS=1
    SIM800_MOD_SAVE_NVM = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+4),///AT&W
    SIM800_MOD_DIS_FUNC = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+5),///AT+CFUNC=0
    SIM800_MOD_EN_FUNC  = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+6),///AT+CFUNC=1
    SIM800_MOD_CHK_SIM  = (SIM800_MOD_CMD_BASE+SIM800_CRITICAL_CMD_BASE+7),///AT+CPIN?
    SIM800_MOD_ECHO_OFF  = (SIM800_MOD_CMD_BASE+SIM800_NORMAL_CMD_BASE+8),///ATE0
};

/** List of Command IDs for Network related commands */
enum SIM800_NET_CMD
{
    SIM800_NET_CRT_CHK      = (SIM800_NET_CMD_BASE+SIM800_CRITICAL_CMD_BASE+1),///AT+CREG?
    SIM800_NET_BSET_CTYPE   = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+2),///AT+SAPBR=3,1,"Contype","GPRS"
    SIM800_NET_BSET_APN     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+3),///AT+SAPBR=3,1,"APN","apn"
    SIM800_NET_BSET_USR     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+4),///AT+SAPBR=3,1,"USER",""
    SIM800_NET_BSET_PWD     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+5),///AT+SAPBR=3,1,"PWD",""
    SIM800_NET_BEN          = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+6),///AT+SAPBR=1,1
    SIM800_NET_BCHK         = (SIM800_NET_CMD_BASE+SIM800_INFO_CMD_BASE+7),///AT+SAPBR=2,1
    SIM800_NET_PDP_CNTX     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+8),///AT+CGDCONT=1,"IP","apn"
    SIM800_NET_ACTV_CID     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+9),///AT+CGACT=1,1
    SIM800_NET_EN_MUX       = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+10),///AT+CIPMUX=1
    SIM800_NET_SEL_DTX      = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+11),///AT+CIPQSEND=1
    SIM800_NET_TSK_START    = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+12),///AT+CSTT="apn","",""
    SIM800_NET_EN_GPRS      = (SIM800_NET_CMD_BASE+SIM800_CRITICAL_CMD_BASE+13),///AT+CIICR
    SIM800_NET_GET_IP       = (SIM800_NET_CMD_BASE+SIM800_INFO_CMD_BASE+14),///AT+CIFSR
    SIM800_NET_CHK          = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+15),//////AT+CREG?
    SIM800_NET_CHK_GPRS     = (SIM800_NET_CMD_BASE+SIM800_NORMAL_CMD_BASE+16)//////AT+GATT?
};

/** List of Command IDs for HTTP related commands */
enum SIM800_HTTP_CMD
{
    SIM800_HTTP_INIT        = (SIM800_HTTP_CMD_BASE+SIM800_NORMAL_CMD_BASE+1),///AT+HTTPINIT
    SIM800_HTTP_PARA_CID    = (SIM800_HTTP_CMD_BASE+SIM800_NORMAL_CMD_BASE+2),///AT+HTTPPARA="CID",1
    SIM800_HTTP_PARA_URL    = (SIM800_HTTP_CMD_BASE+SIM800_NORMAL_CMD_BASE+3),///AT+HTTPPARA="CONTENT","type"
    SIM800_HTTP_PARA_CTYPE  = (SIM800_HTTP_CMD_BASE+SIM800_NORMAL_CMD_BASE+4),///AT+HTTPPARA="URL","you_url.whatever"
    SIM800_HTTP_RQST_GET    = (SIM800_HTTP_CMD_BASE+SIM800_INFO_CMD_BASE+5),///AT+HTTPACTION=0
    SIM800_HTTP_RQST_POST   = (SIM800_HTTP_CMD_BASE+SIM800_INFO_CMD_BASE+6),///AT+HTTPACTION=1
    SIM800_HTTP_DATA_READ   = (SIM800_HTTP_CMD_BASE+SIM800_INFO_CMD_BASE+7),///AT+HTTPREAD
    SIM800_HTTP_DATA_SEND   = (SIM800_HTTP_CMD_BASE+SIM800_INFO_CMD_BASE+8),///AT+HTTPDATA=x,y x:len int, y:timeout int
};

#ifdef __cplusplus
}
#endif

#endif /* SIM800_CMD_ID_H */

/**
 * @}
 * @}
 */