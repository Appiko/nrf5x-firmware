/* 
 * File:   sensebe_rx_detect.h
 * Copyright (c) 2018 Appiko
 * Created on 29 October, 2018, 12:23 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */


#ifndef SENSEBE_RX_DETECT_H
#define SENSEBE_RX_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif
   
/** Structure containing all the values required for sensebe_rx_detect module */
typedef struct 
{
    /** Pin number for Enable pin on TSSP module */
    uint32_t rx_en_pin;
    /** Pin number for Out pin on TSSP module */
    uint32_t rx_out_pin;
    /** Time window after which camera has to be triggered */
    uint32_t time_window_ms;
    /** Pin numbers for out_gen module */
    uint32_t *cam_trig_pin_array;
    /** Initial values for out_gen module */
    bool *out_gen_init_val;
}sensebe_rx_detect_config_t;

/**
 * @brief Function to initialize the Rx detect module
 * @param sensebe_rx_detect_config COnfiguration required for sensbe_rx_detect module
 * @ref sensebe_rx_detect_config_t
 */    
void sensebe_rx_detect_init (sensebe_rx_detect_config_t * sensebe_rx_detect_config);

/**
 * @brief Function to enable detection for SenseBe Rx
 */
void sensebe_rx_detect_start (void);

/**
 * @brief Function to disable detection for SenseBe Rx
 */
void sensebe_rx_detect_stop (void);

/**
 * @brief Function to handle add tick event.
 * @param interval Ticks since the last occurance of add tick event.
 */
void sensebe_rx_detect_add_ticks (uint32_t interval);

#ifdef __cplusplus
}
#endif

#endif /* SENSEBE_RX_DETECT_H */

