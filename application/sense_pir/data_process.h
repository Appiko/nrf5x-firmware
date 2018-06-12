/* 
 * File:   sensepi_pir.h
 * Copyright (c) 2018 Appiko
 * Created on 25 May, 2018, 3:32 PM
 * Author: Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
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
#ifndef OUT_PATTERN_ARRAY_GEN_H
#define OUT_PATTERN_ARRAY_GEN_H

#include "sensepi_ble.h"

///Make this an enum
#define PIR_DATA_PROCESS_MODE false

#define TIMER_DATA_PROCESS_MODE true

#ifdef __cplusplus
extern "C" {
#endif
    
/**
 * @brief Function which is to be called to generate and send output pattern
 * @param data_process_mode boolean value to select from which configuration
 * mode has to be selected
 */
void data_process_pattern_gen(bool data_process_mode);
/**
 * @brief Function to store configuration received from mobile app.
 * @param config Configuration for which local copy has to be created.
 */
void data_process_config(sensepi_config *local_config, uint32_t * out_pin_array);

/**
 * @brief Function which is to be called when Sensepi_PIR module is being \
 * terminated
 */
void data_process_stop();
#ifdef __cplusplus
}
#endif

#endif /* OUT_PATTERN_ARRAY_GEN_H */

