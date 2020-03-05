/*
 *  gps_mod.h : <Write brief>
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
#ifndef GPS_MOD_H
#define GPS_MOD_H

#include "stdint.h"
#include "hal_uarte.h"
#include "nrf_util.h"
#include "device_tick.h"



typedef enum
{
    GPS_MOD_RES_D0 = 1,
    GPS_MOD_RES_D1 = 10,
    GPS_MOD_RES_D2 = 100,
    GPS_MOD_RES_D3 = 1000,
    GPS_MOD_RES_D4 = 10000,
    GPS_MOD_RES_D5 = 100000,
    GPS_MOD_RES_D6 = 1000000,
    GPS_MOD_RES_D7 = 10000000,
}gps_mod_resolution_t;

typedef struct
{
    int32_t lat;
    int32_t lng;
}gps_mod_loc_t;

/** Structure required while initializing */
typedef struct 
{
    /** Enable pin for GSM module */
    uint32_t en_pin;
    /** Baudrate of GSM module | Note:baudrate of log_printf also changes */
    hal_uart_baud_t baudrate;
    /** IRQ priority for UART communication */
    app_irq_priority_t comm_timeout_irq_priority;
    app_irq_priority_t comm_running_irq_priority;
    /** Resolution of GPS Lat Lng output */
    gps_mod_resolution_t resolution;
    /** Function pointer to the function which is to be called on location update */
    void (*loc_handler) (gps_mod_loc_t * current_loc);
    /** Function pointer to the function which will be called on timeout event */
    void (*timeout_handler) (void);
}gps_mod_config_t;

/**
 * @brief Function to be called to initialize the module
 * @param p_mod_init Structure pointer to structure used for initializations
 */
void gps_mod_init (gps_mod_config_t * p_mod_init);

/**
 * @brief Function to start GPS location detect.
 * @param timeout_ms time for which GPS location scan will run
 */
void gps_mod_start (uint32_t timeout_ms);

/**
 * @brief Function to handle add tick event 
 * @param ticks Number of ticks since last add tick events
 */
void gps_mod_add_ticks (uint32_t ticks);

/**
 * @brief Function to start GPS module for indefinite time 
 */
void gps_mod_always_on ();

/**
 * @brief Function to stop GPS location scan
 */
void gps_mod_stop ();

/**
 * @brief Function to get last known location
 * @return Last known location
 */
gps_mod_loc_t * gps_mod_get_last_location ();

/**
 * @brief Abstraction layer for uarte process 
 */
void gps_mod_process ();



#endif /* GPS_MOD_H */
