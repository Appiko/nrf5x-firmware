/*
 *  rf_comm.h : <Write brief>
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


#ifndef RF_COMM_H
#define RF_COMM_H

#include "nrf_util.h"
#include "stdint.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif


//#ifndef RF_COMM_AMPLIFIRE
//#define RF_COMM_AMPLIFIRE
//#endif

#ifndef GPIOTE_CH_USED_RF_COMM_0 
#define GPIOTE_CH_USED_RF_COMM_0 0
#endif

#ifndef GPIOTE_CH_USED_RF_COMM_1 
#define GPIOTE_CH_USED_RF_COMM_1 1
#endif

#ifndef GPIOTE_CH_USED_RF_COMM_2
#define GPIOTE_CH_USED_RF_COMM_2 2
#endif

#ifndef GPIOTE_CH_USED_RF_COMM_3 
#define GPIOTE_CH_USED_RF_COMM_3 3
#endif


typedef enum
{
    RF_EVT_PKT_DONE = 0x01,
    RF_EVT_RADIO_IDLE = 0x02,
    RF_EVT_SYNC_DETECT = 0x04,
            
}rf_comm_events_t;

typedef enum
{
    RF_GPIO0,
    RF_GPIO1,
    RF_GPIO2,
    RF_GPIO3,
}rf_comm_gpio_t;

typedef struct
{
    uint8_t max_len;
    uint8_t app_id;
    uint16_t dev_id;
}rf_comm_pkt_t;

typedef struct
{
    uint32_t rf_reset_pin;
    uint32_t rf_gpio0_pin; ///MCU wakeup
    uint32_t rf_gpio1_pin; ///Sync detected
    uint32_t rf_gpio2_pin; ///CRC Ok
    uint32_t rf_gpio3_pin; ///Collision found
#ifdef RF_COMM_AMPLIFIRE
    uint32_t rf_lna_pin;
    uint32_t rf_pa_pin;
#endif
}rf_comm_hw_t;

/**
 * @brief Structure to set Radio peripheral
 * @{
 */
typedef struct
{
    /**kHz*/
    uint32_t center_freq; 
    /**kHz*/
    uint32_t freq_dev;    
    /**BPS*/
    uint32_t bitrate;
    /** dBm */
    int32_t tx_power;
    /** kHz */
    uint32_t rx_bandwidth;
    app_irq_priority_t irq_priority;
    void (*rf_tx_done_handler) (uint32_t size);
    void (*rf_rx_done_handler) (uint32_t size);
    void (*rf_tx_failed_handler) (uint32_t error);
    void (*rf_rx_failed_handler) (uint32_t error);
}rf_comm_radio_t;
/**
 * @}
 */
/**
 * @brief Function to initialize sub-GHz radio
 * @param p_radio_params structure of Radio parameters
 * @return Status
 * @note Some constant parameters : Preamble, Sync word, Encoding scheme(2 GFSK), AGC
 */
uint32_t rf_comm_radio_init (rf_comm_radio_t * p_radio_params, rf_comm_hw_t * p_comm_hw);

/**
 * @brief Function to set center frequency
 * @param freq Frequency (kHz)
 * @return Status
 */
uint32_t rf_comm_set_freq (uint32_t freq);

/**
 * @brief Function to set BitRate
 * @param bitrate Bit-Rate (bps)
 * @return Status
 */
uint32_t rf_comm_set_bitrate (uint32_t bitrate);

/**
 * @brief Function to set f-dev
 * @param fdev Frequency deviation (kHz)
 * @return Status
 */
uint32_t rf_comm_set_fdev (uint32_t fdev);

/**
 * @brief Function to set Bandwidth
 * @param bandwidth Bandwidth (kHz)
 * @return Status
 */
uint32_t rf_comm_set_bw (uint32_t bandwidth);

/**
 * @brief Function to set transmission power
 * @param pwr Power (dBm -3 to 15)
 * @return Status
 */
uint32_t rf_comm_set_pwr (int32_t pwr);

/**
 * @brief Function to configure the package for application
 * @param pkt_config Pkt Configuration
 * @return Status
 */
uint32_t rf_comm_pkt_config (rf_comm_pkt_t * p_pkt_config);

/**
 * @brief Function to Set packet which to be sent and send
 * @param pkt_type Packet type (maintain, deploy, sense)
 * @param data pointer to actual data
 * @param pkt_len Length of data
 * @return Status
 */
uint32_t rf_comm_pkt_send (uint8_t pkt_type, uint8_t * p_data, uint8_t len);

/**
 * @brief Function to start radio reception.
 * @param p_rxbuff Buffer memory where received data is to be stored.
 * @return Status
 */
uint32_t rf_comm_pkt_receive (uint8_t * p_rxbuff);

/**
 * @brief Function to put radio in idle mode
 * @return Status
 */
uint32_t rf_comm_idle ();

/**
 * @brief Function to put radio in sleep mode
 * @return Status
 */
uint32_t rf_comm_sleep ();

/**
 * @brief Function to get RSSI value of received pkt
 * @return RSSI value of pkt
 */
int32_t rf_comm_get_rssi ();

/**
 * @brief Function to get Product ID of Radio chip which is being used
 * @return Device Product ID (Not I2C address)
 */
uint32_t rf_comm_get_radio_id ();
#endif /* RF_COMM_H */

