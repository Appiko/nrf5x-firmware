/*
 *  ble_adv.h
 *
 *  Created on: 25-Jul-2017
 *
 *  Copyright (c) 2017, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_ble_adv BLE Advertisements
 * @brief Driver of the radio to generate BLE advertisements with scan responses
 *
 * @note This module utilizes the @ref MS_TIMER2 and @ref US_TIMER3, so @ref ms_timer_init
 * and @ref us_timer_init must be called before using this module. Also the radio only
 * works with the high frequency crystal enabled, so @ref hfclk_xtal_init_blocking or
 * @ref hfclk_xtal_init_nonblocking must be used before to enable the it. The radio
 * peripheral uses the highest priority interrupt.
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_BLE_ADV_H_
#define CODEBASE_PERIPHERAL_MODULES_BLE_ADV_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef DEBUG

#define LOG_BUFFER_SIZE     128

typedef struct  {
    uint32_t time;
    const char* func_name;
    uint8_t radio_state;
    uint8_t radio_ctx_state;
    uint8_t freq;
}log_t;

void dump_log(void);

#else

#endif

/**************** Link Layer ****************/

/** Conversion from millisecond to multiples of 0.625 ms */
#define ADV_INTERVAL_MS(x)  ((x*16)/10)

/** @brief length of MAC address in BLE */
#define ADRS_LEN            6
/** @brief length of header in BLE */
#define ADV_HEADER_LEN      2

/** @brief The address types in BLE advertisement: Public and random */
typedef enum {
    PUBLIC_ADRS_PARAM,
    RANDOM_ADRS_PARAM
    /* Page 1279 of 4.2 spec. Controller generates Resolvable Private Address based on the local
    IRK from resolving list. If resolving list contains no matching entry,
    use public/private address. */
    //RESV_PRIV_ADRS_PUBLIC
    //RESV_PRIV_ADRS_PRIVATE
}adrs_type_t;

/** @brief  */

/** @anchor GAP_adv_defines
 * @name The various advertisement formats in an advertisement payload defined
 *  in Generic Access Profile (GAP)
 * @{*/
#define GAP_ADV_FLAGS           0x1
#define GAP_ADV_UUID16_INCOMP   0x2
#define GAP_ADV_UUID16_ALL      0x3
#define GAP_ADV_UUID32_INCOMP   0x4
#define GAP_ADV_UUID32_ALL      0x5
#define GAP_ADV_UUID128_INCOMP  0x6
#define GAP_ADV_UUID128_ALL     0x7
#define GAP_ADV_NAME_SHORT      0x8
#define GAP_ADV_NAME_FULL       0x9
#define GAP_ADV_TRANSMIT_PWR    0xA
#define GAP_ADV_CONN_INTERVAL   0x12
#define GAP_ADV_SERVICE_DATA    0x16
#define GAP_ADV_MANUF_DATA      0xFF
/** @} */

/***** Defines for advertising parameters (BLE Spec 4.2 Vol 2, Part E, 7.8.5 Page No:1277)  *****/
typedef enum {
     ADV_IND_PARAM,             // Connectable undirected advertising
     ADV_DIRECT_IND_PARAM,      // Connectable high duty cycle directed advertising (high duty cycle)
     ADV_SCAN_IND_PARAM,        // Scannable undirected advertising
     ADV_NONCONN_IND_PARAM      // Non connectable undirected advertising
     //ADV_DIRECT_IND           // Connectable low duty cycle directed advertising(low duty cycle)
}ble_adv_type_t;

/** @brief The various advertisement channels commbinations that can be used for advertising */
typedef enum {
    CH_37_PARAM = 1,//!< CH_37_PARAM
    CH_38_PARAM,    //!< CH_38_PARAM
    CH_37_38_PARAM, //!< CH_37_38_PARAM
    CH_39_PARAM,    //!< CH_39_PARAM
    CH_37_39_PARAM, //!< CH_37_39_PARAM
    CH_38_39_PARAM, //!< CH_38_39_PARAM
    CH_ALL_PARAM    //!< CH_ALL_PARAM
}ble_adv_ch_map_t;

/**
 * @brief The structure format for setting the advertisement parameters
 */
typedef struct {
    /** Range: 0x0020 to 0x4000; Time = N * 0.625 msec; Time Range: 20 ms to 10.24 sec */
    uint16_t adv_intvl;
    /** @ref adv_type_param */
    ble_adv_type_t adv_type;
    /** @ref adrs_type_param*/
    adrs_type_t own_adrs_type;
//  Peer_Address_Type,
//  Peer_Address,
    /** @ref ch_map_param.  */
    ble_adv_ch_map_t adv_ch_map;
//  Advertising_Filter_Policy;
} ble_adv_param_t;

/** @brief Set the advertising transmission power in dBm
 *  @param pwr The tx power ranging from -40 to 4
 */
void ble_adv_set_tx_power(int8_t pwr);

/** @brief Get the advertising transmission power in dBm
 *  @return The tx power used
 */
int8_t ble_adv_get_tx_power(void);

/** @brief Start advertising based on the parameters set
 */
void ble_adv_start(void);

/** @brief Stop advertising
 */
void ble_adv_stop(void);

/** @brief Set the advertising data
 *  @param len Length of the advertising data
 *  @param data_ptr Pointer to the buffer containing the data_ptr
 */
void ble_adv_set_adv_data(uint8_t len, uint8_t* data_ptr);

/** @brief Set the advertising parameters to be used
 *  @param adv_param Pointer to the structure containing the parameters
 */
void ble_adv_set_adv_param(ble_adv_param_t * adv_param);

/** @brief Set the random address to be used
 *  @param rand_adrs Pointer to the buffer containing the random address containing
 *                  @ref ADRS_LEN number of octets
 */
void ble_adv_set_random_adrs(uint8_t * rand_adrs);

/** @brief Set the scan response data
 *  @param len Length of the scan response data
 *  @param data_ptr Pointer to the buffer containing the data_ptr
 */
void ble_adv_set_scan_rsp_data(uint8_t len, uint8_t* data_ptr);


#endif /* CODEBASE_PERIPHERAL_MODULES_BLE_ADV_H_ */
/**
 * @}
 * @}
 */

