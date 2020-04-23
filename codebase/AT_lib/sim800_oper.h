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

/**
 * @addtogroup group_at_lib
 * @{
 *
 * @defgroup group_sim800 SIM800 Operations
 * @brief Driver to use SIM800 module using the AT Process
 *
 * @warning This module needs the device_tick to be on and running to be able to work
 * @{
 */

#ifndef SIM800_OPER_H
#define SIM800_OPER_H

#include "stdint.h"

/** List of all possible status of SIM800 module */
typedef enum
{
    /** SIM800 present and idle */
    SIM800_IDLE, 
    /** SIM800 present and executing a command */
    SIM800_BUSY,
    /** SIM800 present and halted. Some critical command failed */
    SIM800_HALT, //(needs reinitialization)
    /** SIM800 present and overloaded. Number of commands exceeded the buffer length */
    SIM800_OVERLOAD, //(needs reinitialization)
    /** SIM800 not present or not connected properly */
    SIM800_NOT_FOUND,
            
}sim800_oper_status_t;


/** State of any network or server connection */
typedef enum
{
    /** Network or Server is disconnected */
    SIM800_DISCONNECTED,
    /** Network or Server is connected */
    SIM800_CONNECTED
}sim800_conn_status_t;

/** For India */
/**JIO doesn't support 2G*/
typedef enum
{
    SIM800_BSNL,
    SIM800_VODAFONE,
    SIM800_IDEA,
    SIM800_AIRTEL,
}sim800_operator_t;

/** Structure to store data needed to send sms */
typedef struct
{
    uint8_t mob_no[10];
    char * body_buff;
    uint8_t body_len;
}sim800_oper_sms_t;

/** Structure to store data needed to establish connection with server
 * @note : Address will be "server"/"resource":"port" 
 * @note : HTTP Timeout is 120s
 */
typedef struct
{
    
    char * server_ptr;
    uint8_t server_len;
    
    char * resource_ptr;
    uint8_t resource_len;
    
    char * port_ptr;    
    uint8_t port_len;   
    
}sim800_server_conn_t;

/** List of HTTP request types */
typedef enum
{
    SIM800_HTTP_GET = 0x30,
    SIM800_HTTP_POST = 0x31,
}sim800_req_type_t;

/** Structure to store information needed to access certain resource from target server */
typedef struct
{
    /** unused for now. Connection number, incase of multiple connections */
    uint32_t conn_id;
    /** HTTP request type */
    sim800_req_type_t req_type;
    /** Payload data pointer (RAW data) */
    uint8_t * payload_ptr;
    /** Length of payload in bytes */
    uint8_t len;
    
}sim800_http_req_t;

/**
 * @brief Function to initialize basic operations of sim800 module
 * @param oper Mobile operator
 * @note use to check if module is presented and connected properly
 */
void sim800_oper_init (sim800_operator_t oper);

/**
 * @brief Function to process SIM800 operations. 
 * @note it has to be called in while(true)
 */
void sim800_oper_process ();

/**
 * @brief Function to handle add ticks event for sim800 operation module
 * @param ticks Number of ticks since last add_tick event
 */
void sim800_oper_add_ticks (uint32_t ticks);

/**
 * @brief Function to get current state of sim800 module
 * @return SIM800 status @ref sim800_oper_status_t
 */
sim800_oper_status_t sim800_oper_get_status ();

/**
 * @brief Function to enable SMS service.
 */
//void sim800_oper_enable_sms (void);

/**
 * @brief Function to enable GPRS service.
 */
void sim800_oper_enable_gprs (void);

/**
 * @brief Function to connect to a server
 * @param conn_params Structure pointer to the structure to store connection parameters 
 * @return Connection ID
 */
uint32_t sim800_oper_conns (sim800_server_conn_t * conn_params);

/**
 * @brief Function to generate http request
 * @param http_req Structure pointer to structure to store http request parameters.
 */
void sim800_oper_http_req (sim800_http_req_t * http_req);

/**
 * @brief Function to get gprs_status
 * @return GPRS connection status
 */
sim800_conn_status_t sim800_oper_get_gprs_status ();

/**
 * @brief Function to get server's connection status
 * @param server_id Server ID
 * @return Server status for given server ID
 */
//sim800_conn_status_t sim800_oper_get_server_status (uint32_t server_id);


#endif /* SIM800_OPER_H */
/**
 * @}
 * @}
 */
