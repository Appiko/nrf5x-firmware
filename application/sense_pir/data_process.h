/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   out_pattern_array_gen.h
 * Author: tejas_tj
 *
 * Created on 25 May, 2018, 4:49 PM
 */

#ifndef OUT_PATTERN_ARRAY_GEN_H
#define OUT_PATTERN_ARRAY_GEN_H

#include "sensepi_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function which is to be called to generate and send output pattern
 */
void data_process_pattern_gen();
/**
 * @brief Function to store configuration received from mobile app.
 * @param config Configuration for which local copy has to be created.
 */
void data_process_local_config_copy(sensepi_config *local_config);
#ifdef __cplusplus
}
#endif

#endif /* OUT_PATTERN_ARRAY_GEN_H */

