/*
 *  dev_id_fw_ver.h
 *
 *  Created on: 17-Aug-2018
 *
 *  Copyright (c) 2018, Appiko
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
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_dev_id_fw_ver Retrieve the device ID and firmware version
 * @brief For a product this module retrieves the device ID programmed in the board
 *  level test at factory and the firmware version provided at compile time.
 * @{
 */


#ifndef CODEBASE_UTIL_DEV_ID_FW_VER_H_
#define CODEBASE_UTIL_DEV_ID_FW_VER_H_

#include "stdint.h"

/**
 * Device ID format specification. All values are in ASCII.
 */
typedef struct
{
    /// Two characters
    /// "SP" for SensePi
    /// "SB" for SenseBe
    uint8_t prod_code[2];
    /// Two numbers (0 to 99) for hardware revision
    uint8_t prod_rev[2];
    /// Factory code (0 to 99)
    /// "00" for Jaaga
    /// "01" for Lion Circuits
    uint8_t factory_code[2];
    /// Last two digits of the year of manufacturing
    uint8_t year[2];
    /// Month of manufacturing
    uint8_t month[2];
    /// Day of manufacturing
    uint8_t day[2];
    /// Serial number of the board being manufactured on that particular day
    /// (0000 to 9999)
    uint8_t serial_no[4];
}__attribute__ ((packed)) dev_id_t ;

/**
 * Firmware version number format as per specification at semver.org
 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t build;
}__attribute__ ((packed)) fw_ver_t ;

/**
 * @brief Gets a pointer to the location where the Device ID is stored
 * @return The said pointer
 * @note The firmware must be compiled by passing the firmware version as
 *  a macro FW_VER at compile time.
 */
dev_id_t * dev_id_get(void);

/**
 * @brief Gets a pointer to the location where the Device ID is stored
 * @return The said pointer
 * @note The device must be programmed with the device ID in UICR 0 to 3
 *  during manufacturing
 */
fw_ver_t * fw_ver_get(void);

#endif /* CODEBASE_UTIL_DEV_ID_FW_VER_H_ */

/**
 * @}
 * @}
 */
