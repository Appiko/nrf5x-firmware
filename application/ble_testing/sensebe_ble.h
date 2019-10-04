/*
 *  sensebe_ble.h : BLE Support file for SenseBe application 
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

/**
 * @addtogroup sensebe_appln
 * @{
 *
 * @defgroup ble_support The support code for the active IR based Sense units.
 * @brief The active IR sense application's support file that handles ble operations.
 *
 * @{
 *
 */

#ifndef APPLICATION_SENSEBE_BLE_H_
#define APPLICATION_SENSEBE_BLE_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"
#include "dev_id_fw_ver.h"

typedef struct {
    dev_id_t id;
    uint8_t battery_status;
    fw_ver_t fw_ver;
} __attribute__((packed)) sensebe_sysinfo;

/**< Name of device, to be included in the advertising data. */

#define DEVICE_NAME_CHAR 'S', 'e', 'n', 's', 'e', 'B', 'e', ' ', 'R', 'x'

/** Complete 128 bit UUID of the SenseBe service
 * 3c73dc60-07f5-480d-b066-837407fbde0a */
#ifdef DEVICE_UUID_COMPLETE
#define SENSEBERX_UUID_COMPLETE \
    {                           \
        DEVICE_UUID_COMPLETE    \
    }
#else
#define SENSEBERX_UUID_COMPLETE                                                                        \
    {                                                                                                  \
        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x60, 0xdc, 0x73, 0x3c \
    }
#endif
/** The 16 bit UUID of the Sense Be service */
#ifdef DEVICE_UUID_SERVICE
#define SENSEBERX_UUID_SERVICE DEVICE_UUID_SERVICE
#else
#define SENSEBERX_UUID_SERVICE 0xdc60
#endif
/** The 16 bit UUID of the read-only System Info characteristic */
#ifdef DEVICE_UUID_SYSINFO
#define SENSEBERX_UUID_SYSINFO DEVICE_UUID_SYSINFO
#else
#define SENSEBERX_UUID_SYSINFO 0xdc61
#endif
/** The 16 bit UUID of the read-write Config characteristic */
#ifdef DEVICE_UUID_CONFIG
#define SENSEBERX_UUID_CONFIG DEVICE_UUID_CONFIG
#else
#define SENSEBERX_UUID_CONFIG 0xdc62
#endif

typedef struct {
    uint32_t fw_ver_int;
    dev_id_t dev_id;
    uint8_t battery_voltage;
}__attribute__((packed))  senseberx_sysinfo;

/** List of different battery types */
typedef enum {
    BATTERY_STANDARD,
    BATTERY_RECHARAGEABLE,
} battery_type_t;

/** List of all possible device speed. (sampling speed or sys_wakeup time)
     The interval at which IR pulses are sent from the Tx unit
     This should be same on the Tx and Rx unit. Reduce this to save power but reduce sensitivity */
typedef enum {
    SPEED_LIGHTNING, /// 5 ms
    SPEED_FAST, /// 25 ms
    SPEED_NORMAL, /// 100 ms
    SPEED_SLOW, /// 250 ms
} device_speed_t;

/** List of all possible trigger combination. */
typedef enum {
    /** Motion */
    MOTION_ONLY,
    /** Timer */
    TIMER_ONLY,
    /** Both */
    MOTION_AND_TIMER,
    /** None */
    NONE,
} triggers_t;

/** For either Motion (radio) or Timer, when should it be operational */
typedef enum {
    /** No cut-off condition */
    ALL_TIME,
    /** Time of day for cut-off */
    TIME_OF_DAY,
    /** Based on Light Condition */
    AMBIENT_LIGHT,
} oper_cond_sel_t;

/** Used only when the variable with oper_cond_sel_t is set to AMBIENT_LIGHT 
     This uses the ambient light sensor in the device */
typedef enum {
    /** When Light is more than threshold */
    DAY_ONLY,
    /** When Light is less than threshold */
    NIGHT_ONLY,
    /** When Light is more than threshold */
    DAY_AND_TWILIGHT,
    /** When Light is less than threshold */
    NIGHT_AND_TWILIGHT,
} ambi_lgt_cond_t;

/** List of all radio channels */
typedef enum {
    CHANNEL0, /// 2497 MHz
    CHANNEL1, /// 2489 MHz
    CHANNEL2, /// 2483 MHz
    CHANNEL3, /// 2479 MHz
    CHANNEL4, /// 2473 MHz
    CHANNEL5, /// 2471 MHz
    CHANNEL6, /// 2467 MHz
    CHANNEL7, /// 2461 MHz
    CHANNEL8, /// 2459 MHz
    CHANNEL9, /// 2453 MHz
    MAX_CHANNEL,
} channels_t;

/** List of all settings present in one BLE configuration */
typedef enum {
    SETTINGS0,
    SETTINGS1,
    SETTINGS2,
    SETTINGS3,
    SETTINGS4,
    SETTINGS5,
    SETTINGS6,
    SETTINGS7,
    MAX_SETTINGS,
} settings_list_t;

/** Day of time selection
00:00 is 0 and max count is 86400s (for 24 hours) */
typedef struct {
    /** Start Time for operation in sec.s  */
    uint32_t start_time;
    /** End Time for operation in sec.s */
    uint32_t end_time;
}__attribute__((packed))  time_cond_t;

/** Day Light condition selection */
typedef struct {
    /**Device will be working if ambient light is greater than this threshold*/
    /** Three thresholds x1,x2,x3 => x3 > x2 > x1 */
    ///All day   :  0
    ///Day Only   :  x2
    ///Night Only   :  0
    ///Twilight only : x1
    ///Day & Twilight   :  x1
    ///Night & Twilight   :  0
    uint32_t lower_light_threshold;
    ///All day   :  0xFFFFFFFF
    ///Day Only   :  0xFFFFFFFF
    ///Night Only   :  x2
    ///Twilight only : x3
    ///Day & Twilight   :  0xFFFFFFFF
    ///Night & Twilight   :  x3
    /** Device will be working if ambient light is lower than this threshold */
    uint32_t higher_light_threshold;
}__attribute__((packed))  light_cond_t;

typedef union {
    time_cond_t time_cond;
    light_cond_t light_cond;
}__attribute__((packed))  oper_cond_t;

typedef enum {
    SINGLE_SHOT,
    MULTI_SHOT,
    LONG_PRESS,
    VIDEO,
    HALF_PRESS,
    NO_SHOT,
}__attribute__((packed))  trigger_mode_t;

typedef struct {
    uint16_t num_shots; /// 2 to 32 shots
    uint16_t shot_interval_100ms; /// 0.5s to 100 min
}__attribute__((packed))  multishot_setting_t;

typedef struct {
    uint16_t video_duration_1s; /// 1s to 1000 min
    uint8_t extension_time; /// 1s to 250 s
    uint8_t num_extensions; /// 1 to 20
}__attribute__((packed))  video_setting_t;

/** Shared memory for mode settings */
typedef union {
    uint32_t long_press_duration_100ms; /// Max of 24 hours
    multishot_setting_t multishot;
    video_setting_t video;
}__attribute__((packed))  mode_settings_t;

/** Structure to store camera trigger */
typedef struct //older structure as it it
{
    /** Pre-Focus Enable bit */
    uint8_t pre_focus_en : 1;
    /** Video with full press signal enable, half press when 0 */
    uint8_t video_w_full_press_en : 1;
    /** Radio enable bit */
    uint8_t radio_trig_en : 1;
    /** Camera trigger mode */
    trigger_mode_t mode : 5;
    /** Settings based on the mode */
    mode_settings_t mode_setting;
    /** Pulse duration of trigger and focus signal in 100ms, 25s max, 300 ms default */
    /** Editable in single shot, multi shot, video and half-press mode  */
    uint8_t trig_pulse_duration_100ms; //1Byte
    /** Pre focus signal duration in 100ms, 25 sec max, 200 ms default */
    /** (pre-focus is generally used to wake-up the camera or to use auto-focus in camera ) */
    uint8_t prf_pulse_duration_100ms; //1Byte
}__attribute__((packed))  cam_trig_t;

/** Structure to store camera operational settings common for both MOTION and TIMER trigger */
typedef struct {
    /** Camera trigger  configuration */
    cam_trig_t cam_trigger; //7Bytes
    /** Operational condition selection (TOD/Ambient)*/
    oper_cond_t oper_cond; //8Bytes
}__attribute__((packed))  cam_settings_t; ///15Bytes

/** Motion settings */
typedef struct {
    /** Variable to enable/disable Beam Rx */
    uint8_t is_enable;

    /** Number of times the trigger happens as long as a beam is cut. Inter trigger timer defines the interval between the triggers. Min 1 and max is 250 */
    uint8_t detect_trigger_num;

    /** Variable to decide sensitivity */
    uint16_t sensitivity; //Sensitivity is the time for which beam is to be blocked.

    /** Downtime, once motion is detected */
    uint16_t inter_trig_time;
}__attribute__((packed))  detection_functionality_t; //6 Bytes

/** Shared memory for functionality selection */
typedef union {
    detection_functionality_t detection_func;
    uint32_t timer_duration;
}__attribute__((packed))  function_settings_t; //6 Bytes

/** Generic structure to store trigger settings */
typedef struct {
    /** Trigger selection */
    triggers_t trig_sel; //1Byte
    /** Functional Settings */
    function_settings_t func_setting; //6Bytes
    /** Camera settings */
    cam_settings_t cam_setting; //15Bytes
}__attribute__((packed))  settings_t; //22Bytes

/** Structure to store radio triggering parameters */
typedef struct {
    /** Frequency channel used */
    channels_t radio_channel; //1Byte
    /** Radio on time in 25ms units. Default is 4 (100 ms). Ranges from 2 to 20*/
    uint8_t radio_oper_duration_25ms; //1Byte
    /** Radio operation frequency(time period after which radio operation is to be repeated) */
    uint8_t radio_oper_freq_100us; //1Byte
}__attribute__((packed))  radio_control_t;

/** Structure to store date in ble data */
typedef struct {
    /** Day */
    uint8_t dd;
    /** Month */
    uint8_t mm;
    /** Year */
    uint8_t yy;
} __attribute__((packed)) date_ble_t;

/** BLE structure */
typedef struct {
    /** Array of operation condition selection for each trigger */
    oper_cond_sel_t trigger_oper_cond_sel[MOTION_AND_TIMER]; //2Bytes
    /** Array of generic settings */
    settings_t generic_settings[MAX_SETTINGS]; //176Bytes
    /** Radio control */
    radio_control_t radio_control; //3Bytes
    /** System speed */
    device_speed_t speed; //1Byte
    /** Battery type */
    battery_type_t battery_type; //1Byte
    /** User device name */
    uint8_t dev_name[16]; //16Bytes
    /** Current time in sec.s */
    uint32_t current_time; //4Bytes
    /** Current Date in dd/mm/yy format */
    date_ble_t current_date; //3Bytes
} __attribute__((packed)) senseberx_config_t; ///206Bytes

typedef struct {
    uint8_t *adv_data;
    uint16_t adv_len;
    uint8_t *scan_rsp_data;
    uint16_t scan_rsp_len;
} senseberx_ble_adv_data_t;

/**
 * @brief Initialize the handlers to pass the BLE SD events
 *  and the configuration received from the mobile app
 * @param ble_sd_evt Handler to send the BLE events to the application
 * @param config_update Handler to send SensePi config to the application
 */
void senseberx_ble_init(void (*ble_sd_evt)(ble_evt_t *evt),
        void (*config_update)(senseberx_config_t *cfg));

/**
 * @brief Updates the characteristic that stores the sysinfo
 * @param sysinfo A pointer to the Sense Pi info for the updation
 */
void senseberx_ble_update_sysinfo(sensebe_sysinfo *sysinfo);

/**
 * @brief Updates the characteristic that stores the SensePi config
 * @param config A pointer to the new Sense Pi configuration
 */
void senseberx_ble_update_config(senseberx_config_t *config);

/**
 * @brief Disconnect the current active connection, if already connected
 */
void senseberx_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void senseberx_ble_stack_init(void);

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void senseberx_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void senseberx_ble_gap_params_init(void);

/**
 * @brief Function to initializing the advertising
 * @param sensebe_ble_adv_data Advaertise data and scan response data along with
 * their respective lengths.
 */
void senseberx_ble_adv_init(senseberx_ble_adv_data_t *sensebe_ble_adv_data);

/**
 * @brief Function to start advertising.
 */
void senseberx_ble_adv_start(void);

#endif /* APPLICATION_SENSEBE_BLE_H_ */

/**
 * @}
 * @}
 */
