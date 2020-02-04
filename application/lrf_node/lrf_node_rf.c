/*
 *  lrf_node_rf.c : <Write brief>
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

//Includes
#include "lrf_node_rf.h"
#include "common_util.h"
#include "ms_timer.h"
#include "KXTJ3.h"
#include "aa_aaa_battery_check.h"
#include "random_num.h"
#include "hal_gpio.h"
#include "string.h"
#include "log.h"


//macros
/** MS Timer used by this module */
#define MOD_TIMER CONCAT_2(MS_TIMER, MS_TIMER_USED_LRF_NODE_MOD)
#define S_to_MS(x)  ( 1000* (x))
/** Accelerometer frequency in sensing state */
#define SENSE_FREQ_S (1)
#define SENSE_FREQ_MS (S_to_MS(SENSE_FREQ_S))
/** Accelerometer frequency in tilted state */
#define TILT_FREQ_S (3)
#define TILT_FREQ_MS (S_to_MS(TILT_FREQ_S))
/** Accelerometer frequency in maintain state */
#define MAINTAIN_FREQ_S (7)
#define MAINTAIN_FREQ_MS (S_to_MS(MAINTAIN_FREQ_S))
/** Alive signal frequency. Alive signal is to be sent if no other package has been sent */
#define ALIVE_FREQ_S (30)

/** Threshold acceleration for motion */
#define MOTION_THRESHOLD 1200
/** Tilt threshold angle */
#define TILT_THRESHOLD 30
/** Maximum number of packet before state changes to maintain */
#define MAX_NUM_TILT 3

/** Maximum length of RF packet */
#define RF_MAX_LEN 16

/** PKT Structure */
typedef struct
{
    uint8_t batt_volt;
    uint8_t angle;
}node_rf_pkt_t;

//typedefs
typedef enum
{
    PKT_ALIVE,
    PKT_SENSE,
    PKT_MAINTAIN,
    PKT_INVALID,
}pkt_types_t;

typedef enum 
{
    STATE_SENSE,
    STATE_TILT,
    STATE_MAINTAIN,
    STATE_INVALID,
}node_states_t;


//unchanging variables

//global variables
/**Flag to keep track if node is tilted or not*/
static bool g_node_is_tilted = false;
/** Variable storing current tilt angle of node */
static uint32_t g_node_current_angle = 0;
/** Variable to store latest acceleration value */
static KXTJ3_g_data_t g_acce_data;
///** Variable to store Node header */
//static lrf_node_mod_rf_head_t g_rf_header;
///** Variable to store RF parameters */
//static lrf_node_mod_rf_params_t g_rf_params;
/** Variable to keep track of packet count */
static uint32_t g_pkt_cnt = 0;
/** Variable to keep track of  */
static node_states_t g_node_state;
/** Variable to keep track of packet type to  be sent */
static pkt_types_t g_current_pkt_type = PKT_ALIVE;
/** Variable to store threshold angle */
static uint8_t g_node_threshold_angle = 30;
/** Array to store ms_timer_ticks for any state */
static uint64_t garr_ms_ticks[] = {MS_TIMER_TICKS_MS(SENSE_FREQ_MS), 
                                   MS_TIMER_TICKS_MS(TILT_FREQ_MS),
                                   MS_TIMER_TICKS_MS(MAINTAIN_FREQ_MS)};
/** Array to store frequency of different states in S */
static uint32_t garr_freq_s[] = {SENSE_FREQ_S, TILT_FREQ_S, MAINTAIN_FREQ_S};
/** Array to store random time offset for given state */
static uint32_t garr_random_offset [STATE_INVALID];

/**Variable to store TCXO pin number */
static uint32_t g_pin_tcxo_en;


/**variable to store RF HW information*/

void node_rf_tx_done (uint32_t status);

void node_rf_tx_failed (uint32_t status);

static rf_comm_radio_t g_rf_comm_radio = 
{
    .rf_tx_done_handler = node_rf_tx_done,
    .rf_tx_failed_handler = node_rf_tx_failed,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
};

static rf_comm_hw_t g_rf_comm_hw;

//function declaration

/**
 * @brief Function to wakeup radio from sleep
 */
void node_rf_wakeup ();

/**
 * @brief Function to put radio to sleep
 */
void node_rf_sleep ();

/**
 * @brief Function to measure angle from reference axis
 * @param acce_comp Acceleration component along reference axis
 * @return Angle from reference axis
 */
uint8_t get_angle (uint32_t acce_comp);

/**
 * @brief Function to check if net acceleration in below threshold level.
 * @param threshold Threshold on net acceleration (in milli g)
 * @return Node's motion status
 * @retval 1 if acceleration is greater than or equal to threshold
 * @retval 0 if acceleration is less than threshold value 
 */
bool is_node_moving (uint32_t threshold);

/** Common timer handler. For each state change time duration will get updated */
void ms_timer_handler (void);

void state_sense_handler (void);
void state_tilt_handler (void);
void state_maintain_handler (void);

/**
 * @brief Function to manage switching between states.
 */
void switch_manage ();

//function array

//function definitions

const uint32_t gc_cos_res5d[] = {
    1000, 996, 984, 965, 939, 906, 866, 819, 766,
    707, 642, 573, 500, 422, 342, 258, 173, 87, 0
};

uint8_t get_angle (uint32_t acce_comp)
{
    acce_comp = acce_comp < 0 ? acce_comp*(-1) : acce_comp;
    uint8_t angle_cnt = 0;
    while((gc_cos_res5d[angle_cnt] > acce_comp) && (angle_cnt < 19))
    {
        angle_cnt++;
    }
    return ((angle_cnt)*5);
}

bool is_node_moving (uint32_t threshold)
{
    uint32_t res = (g_acce_data.xg*g_acce_data.xg) 
        + (g_acce_data.yg * g_acce_data.yg) + (g_acce_data.zg*g_acce_data.zg);
    if(res  <= (threshold * threshold))
    {
        return 1;
    }
    else
    {
        return 0;
    }}

void node_rf_wakeup ()
{
    hal_gpio_pin_set (g_pin_tcxo_en);
    rf_comm_wake();
    rf_comm_radio_init (&g_rf_comm_radio, &g_rf_comm_hw);

    rf_comm_idle ();
    
}

void node_rf_sleep ()
{
    rf_comm_sleep ();
    hal_gpio_pin_clear (g_pin_tcxo_en);
}

void state_sense_handler (void)
{
    if(g_node_is_tilted)
    {
        g_node_state = STATE_TILT;
        g_current_pkt_type = PKT_SENSE;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate
            (S_to_MS(1),S_to_MS(5))));
    }
    else
    {
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate (0,1000)));
        g_current_pkt_type = PKT_ALIVE;
        g_pkt_cnt = 0;        
    }
//    log_printf ("%s\n",__func__);
}

void state_tilt_handler (void)
{
    if(g_node_is_tilted)
    {
        g_current_pkt_type = PKT_SENSE;
        g_pkt_cnt++;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate 
            (S_to_MS(1),S_to_MS(5))));
    }
    else
    {
        g_node_state = STATE_SENSE;
        g_pkt_cnt = 0;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate (0,1000)));
        
    }
    if(g_pkt_cnt >= MAX_NUM_TILT)
    {
        g_node_state = STATE_MAINTAIN;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate
            (S_to_MS(1),S_to_MS(5))));
    }
//    log_printf ("%s\n",__func__);
}

void state_maintain_handler (void)
{
    if(g_node_is_tilted)
    {
        g_current_pkt_type = PKT_MAINTAIN;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate
            (S_to_MS(10),S_to_MS(30))));
    }
    else
    {
        g_current_pkt_type = PKT_ALIVE;
        g_node_state = STATE_SENSE;
        g_pkt_cnt = 0;
        garr_random_offset[g_node_state] = MS_TIMER_TICKS_MS ((random_num_generate (0,1000)));
    }
//    log_printf ("%s\n",__func__);
}

void switch_manage ()
{
    switch (g_node_state)
    {
        case STATE_SENSE : 
            state_sense_handler ();
            break;
            
        case STATE_TILT : 
            state_tilt_handler ();
            break;
            
        case STATE_MAINTAIN : 
            state_maintain_handler ();
            break;
            
        case STATE_INVALID : 
//            state_sense_handler ();
            break;
            
    }
}

void ms_timer_handler (void)
{
//    log_printf ("%s\n",__func__);
    //calculate angle
    static uint32_t l_sense_alive_s = 0;
    memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));
    g_node_current_angle = get_angle (g_acce_data.xg);
    bool l_current_motion = is_node_moving (MOTION_THRESHOLD);
    //check if node is tilted
    if ((g_node_current_angle > g_node_threshold_angle) && l_current_motion)
    {
        g_node_is_tilted = true;
    }
    else
    {
        g_node_is_tilted = false;
    }
    //switch management
    switch_manage ();
    
    //start new ms_timer with random offset
    ms_timer_start (MOD_TIMER, MS_SINGLE_CALL,
                    (garr_ms_ticks[g_node_state] + garr_random_offset[g_node_state]),
                    ms_timer_handler);
    
    //send RF packet if needed
    if((l_sense_alive_s < ALIVE_FREQ_S) && (g_node_state == STATE_SENSE))
    {
        l_sense_alive_s += (garr_freq_s[g_node_state] + 
            ROUNDED_DIV(garr_random_offset[g_node_state], 32768));
        
        
    }
    else
    {
        node_rf_wakeup ();
        l_sense_alive_s = 0;
        node_rf_pkt_t l_pkt = 
        {
            .batt_volt = aa_aaa_battery_status (),
            .angle = g_node_current_angle
        };
        //RF
        rf_comm_pkt_send (g_current_pkt_type, (uint8_t *)&l_pkt, 
                          sizeof(node_rf_pkt_t));
    }
    
}

void node_rf_tx_done (uint32_t status)
{
    log_printf("Tx Done : %d\n", status);
    node_rf_sleep ();
}

void node_rf_tx_failed (uint32_t status)
{
    //in certain cases
    log_printf("Tx Failed : %d\n", status);
//    if(status != 0)
//    {
//        node_rf_pkt_t l_pkt = 
//        {
//            .batt_volt = aa_aaa_battery_status (),
//            .angle = g_node_current_angle
//        };
//        //RF
//        rf_comm_pkt_send (g_current_pkt_type, (uint8_t *)&l_pkt, 
//                          sizeof(node_rf_pkt_t));
//    }
//    else
    {
//        rf_comm_flush ();
        node_rf_sleep ();

    }
}

/** APIs */
void lrf_node_mod_init (lrf_node_mod_init_t * p_mod_init)
{
    //initialize RF comm module
    g_rf_comm_radio.bitrate = p_mod_init->radio_params.bitrate;
    g_rf_comm_radio.center_freq = p_mod_init->radio_params.center_freq;
    g_rf_comm_radio.freq_dev = p_mod_init->radio_params.fdev;
    g_rf_comm_radio.tx_power = p_mod_init->radio_params.tx_power;
    
    memcpy (&g_rf_comm_hw, &p_mod_init->radio_gpio, sizeof(rf_comm_hw_t));
    
    rf_spi_init ((rf_spi_init_t * )&p_mod_init->radio_spi);
    //Save RF packet header
    rf_comm_pkt_t l_rf_head = 
    {
        .max_len = RF_MAX_LEN,
        .app_id = p_mod_init->radio_header.app_id,
        .dev_id = p_mod_init->radio_header.prod_id,
    };
    rf_comm_pkt_config (&l_rf_head);
    
    g_pin_tcxo_en = p_mod_init->rf_tcxo_pin;
    hal_gpio_cfg_output (g_pin_tcxo_en, 0);
    
    //initialize accelerometer
    g_node_threshold_angle = p_mod_init->threshold_angle;
    KXTJ3_config_t l_acce_init = 
    {
        .i2c_sda = p_mod_init->acce_hw_params.SDA,
        .i2c_sck = p_mod_init->acce_hw_params.SCK,
        .range = KXTJ_RNG_2g,
        .resolution = KXTJ_RES_8Bit,
    };
    kxtj3_init (&l_acce_init);
    kxtj3_start ();
    //
}

void lrf_node_mod_start ()
{
    //start sensing the angle
    g_node_is_tilted = false;
    g_node_current_angle = 0;
    g_node_state = STATE_SENSE;
    g_pkt_cnt = 0;
    g_acce_data.xg = 0;
    g_acce_data.yg = 0;
    g_acce_data.zg = 0;
    ms_timer_start (MOD_TIMER, MS_SINGLE_CALL,
                    (garr_ms_ticks[g_node_state] + garr_random_offset[g_node_state]),
                    ms_timer_handler);
    
}

void lrf_node_mod_stop ()
{
    //stop everything except accelerometer
    ms_timer_stop (MOD_TIMER);
    g_node_is_tilted = false;
    g_node_current_angle = 0;
    g_node_state = STATE_INVALID;
    g_pkt_cnt = 0;
    g_acce_data.xg = 0;
    g_acce_data.yg = 0;
    g_acce_data.zg = 0;
}

void lrf_node_mod_update_rf_head (lrf_node_mod_rf_head_t * p_head)
{
    rf_comm_pkt_t l_rf_pkt = 
    {
        .max_len = RF_MAX_LEN,
        .dev_id = p_head->prod_id,
        .app_id = p_head->app_id,
    };
    rf_comm_pkt_config (&l_rf_pkt);
}

void lrf_node_mod_update_rf_params (lrf_node_mod_rf_params_t * p_params)
{
    //reinitialize RF comm with new params
    g_rf_comm_radio.bitrate = p_params->bitrate;
    g_rf_comm_radio.center_freq = p_params->center_freq;
    g_rf_comm_radio.freq_dev = p_params->fdev;
    g_rf_comm_radio.tx_power = p_params->tx_power;
}

void lrf_node_mod_set_angle_threshold (uint8_t angle)
{
    //save threshold angle into global parameters
    g_node_threshold_angle = angle;
}


uint8_t lrf_node_mod_get_angle ()
{
    //return current angle
    memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));
    g_node_current_angle = get_angle (g_acce_data.xg);
    return g_node_current_angle;
}

uint8_t lrf_node_mod_is_tilted ()
{
    //return tilt status
    memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));
    g_node_current_angle = get_angle (g_acce_data.xg);
    bool l_current_motion = is_node_moving (MOTION_THRESHOLD);
    if ((g_node_current_angle > g_node_threshold_angle) && l_current_motion)
    {
        g_node_is_tilted = true;
    }
    else
    {
        g_node_is_tilted = false;
    }
    return g_node_is_tilted;
}
