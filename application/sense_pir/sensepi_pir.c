/* 
 * File:   sensepi_pir.c
 * Copyright (c) 2018 Appiko
 * Created on 1 June, 2018, 3:30 PM
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


#include "sensepi_pir.h"

#include "data_process.h"
#include "sensepi_ble.h"

#include "pir_sense.h"
#include "led_sense.h"
#include "log.h"
#include "device_tick.h"
#include "mcp4012_x.h"
#include "ms_timer.h"

#include "boards.h"

#include "hal_pin_analog_input.h"
#include "hal_nop_delay.h"

#include "nrf_util.h"

#include "stdint.h"
#include "stdbool.h"
#include "string.h"


#define LED_WAIT_TIME_MS 301
#define PIR_SENSE_INTERVAL_MS 50
#define PIR_SENSE_THRESHOLD 600
#define PIR_THRESHOLD_MULTIPLY_FACTOR 8
#define LIGHT_THRESHOLD_MULTIPLY_FACTOR 32
#define SENSEPI_PIR_SLOW_TICK_INTERVAL_MS 300000

//Not a pointer, but a variable that gets the config copied to
static sensepi_pir_config_t config;
static bool pir_sense_flag = 1;
static bool use_timer = false;
static pir_sense_cfg config_pir;
static uint32_t intr_trig_time_in = 0;
static uint32_t intr_trig_time_count = 0;
static uint32_t timer_interval_count = 0;
static uint32_t timer_interval_in = 0;
static uint32_t working_mode = 0;
//static bool timer_done_flag = 0;
sensepi_config current_config;

/**Enum with all the substate for SENSING state*/
typedef enum
{
    PIR_ENABLED,
    PIR_DISABLED,
}sensepi_pir_substate;

static sensepi_pir_substate pir_substate = PIR_ENABLED;
/**
 * @brief Function to enable PIR sensing
 */
void pir_enable();

/**
 * @brief Function to disable PIR sensing
 */
void pir_disable();

/**
 * @brief Function which will handle PIR operations.
 * @param interval Interval value sent from main application
 */
void pir_operation(uint32_t interval);

/**IRQ Handler for PIR interrupt*/
void pir_handler(int32_t adc_val);

void timer_handler(void);

/**
 * @brief Function to enable LED sensing with proper parameters.
 * @param led_conf
 */
void led_sense_conf(sensepi_pir_config_t * led_conf);

/**
 * @brief Function to check light conditions and comapre light conditions with 
 * configuration provided by user.
 * @param oper_time_temp Local copy of oper_time to select light condition
 * configuration
 * @return if light conditions satisfies the conditions provided by config
 * return 1, else retunr 0.
 */
bool light_check(oper_time_t oper_time_temp);

void pir_enable()
{
    log_printf("PIR_Enabled\n");
    pir_sense_start(&config_pir);
    pir_sense_flag = 1;
    device_tick_switch_mode(DEVICE_TICK_SLOW);
    pir_substate = PIR_ENABLED;

    ///The 300 ms delay needs to be here.
    return;
}

void pir_disable()
{
    log_printf("PIR_Disabled\n");
    pir_sense_stop();
    pir_substate = PIR_DISABLED;
    intr_trig_time_count = 0;
    pir_sense_flag = 0;
}

void pir_operation(uint32_t interval)
{
    log_printf("PIR_Operation\n");
    switch(pir_substate)
    {
        case PIR_ENABLED:
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time) == false)
            {
                pir_substate = PIR_DISABLED;
            }
            break;
        }
        case PIR_DISABLED :
        {
            if(light_check(config.config_sensepi->pir_conf->oper_time) == true)
            {
                intr_trig_time_count += interval;
                if(intr_trig_time_count >= intr_trig_time_in)
                {
                    pir_enable();
                }
            }
            break;
        }
    }

}

void pir_handler(int32_t adc_val)
{
    log_printf("Sensed %d\n", adc_val);
    device_tick_switch_mode(DEVICE_TICK_FAST);
    data_process_pattern_gen(PIR_DATA_PROCESS_MODE);
    if(pir_sense_flag == 1)
    {
        pir_disable();
    }
}

void timer_handler(void)
{
    log_printf("Timer Handler\n");
    data_process_pattern_gen(TIMER_DATA_PROCESS_MODE);
    ms_timer_stop(MS_TIMER1);
}

void led_sense_conf(sensepi_pir_config_t * led_conf)
{
    log_printf("Led_Sense_Conf\n");
    led_sense_init(led_conf->led_sense_out_pin,
            led_conf->led_sense_analog_in_pin,
            led_conf->led_sense_off_val);
}

bool light_check(oper_time_t oper_time_temp)
{
    log_printf("Light check\n");
    log_printf("Light intensity : %d", led_sense_get());
    static uint8_t light_sense_config = 1;
    static uint32_t light_threshold = 0;
    static bool light_check_flag = 0;
    light_sense_config = oper_time_temp.day_or_night;
    light_threshold = (uint32_t)((oper_time_temp.threshold) * LIGHT_THRESHOLD_MULTIPLY_FACTOR);
    if(light_sense_config == 1 && led_sense_get() >= light_threshold)
    {
        light_check_flag = 1;
    }
    else if(light_sense_config == 0 && led_sense_get() <= light_threshold)
    {
        light_check_flag = 1;
    }
    else
    {
        light_check_flag = 0;
        device_tick_switch_mode(DEVICE_TICK_SLOW);
    }
    return light_check_flag;

}

void sensepi_pir_start()
{
    log_printf("SensePi_PIR_Start\n");
    led_sense_cfg_input(true);
    hal_nop_delay_ms(LED_WAIT_TIME_MS);
    timer_interval_in = config.config_sensepi->timer_conf->timer_interval;
    if( timer_interval_in < SENSEPI_PIR_SLOW_TICK_INTERVAL_MS)
    {
        ms_timer_start(MS_TIMER1,
            MS_REPEATED_CALL,
            LFCLK_TICKS_977(config.config_sensepi->timer_conf->timer_interval),
            timer_handler);
        use_timer = true;
    }
}

void sensepi_pir_stop()
{
    log_printf("SensePi_PIR_Stop\n");
    led_sense_cfg_input(false);
    pir_disable();
    ms_timer_stop(MS_TIMER1);
    use_timer = false;
}

void sensepi_pir_init(sensepi_pir_config_t * config_sensepi_pir)
{
    log_printf("SensePi_PIR_init\n");
    memcpy(&config, config_sensepi_pir, sizeof(config));
    sensepi_config local_sensepi_config;
    memcpy(&local_sensepi_config, config.config_sensepi,sizeof(local_sensepi_config));
    working_mode = local_sensepi_config.trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config.pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = local_sensepi_config.pir_conf->intr_trig_timer;
            led_sense_conf(&config);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_pir->config_sensepi->pir_conf->amplification);
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = local_sensepi_config.timer_conf->timer_interval;
            break;
        }
        case PIR_AND_TIMER:
        {
            pir_sense_cfg local_config_pir = 
            {
                PIR_SENSE_INTERVAL_MS, config.pir_sense_signal_input,
                config.pir_sense_offset_input,
                ((uint32_t)local_sensepi_config.pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR,
                APP_IRQ_PRIORITY_HIGH, pir_handler, 
            };
            config_pir = local_config_pir;
            intr_trig_time_in = local_sensepi_config.pir_conf->intr_trig_timer;
            led_sense_conf(&config);
            mcp4012_init(config.amp_cs_pin, config.amp_ud_pin, config.amp_spi_sck_pin);
            mcp4012_set_value(config_sensepi_pir->config_sensepi->pir_conf->amplification);
            timer_interval_in = local_sensepi_config.timer_conf->timer_interval;
            break;
        }
    data_process_config(&local_sensepi_config, config.signal_out_pin_array);

    }
    return;
}

void sensepi_pir_update(sensepi_config * update_config)
{
    log_printf("SensePi_PIR_update\n");
    working_mode = update_config->trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            config_pir.threshold = ((uint32_t) update_config->pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(update_config->pir_conf->amplification);
            intr_trig_time_in = (uint32_t)update_config->pir_conf->intr_trig_timer;
            break;
        }
        case TIMER_ONLY:
        {
            timer_interval_in = update_config->timer_conf->timer_interval;
            break;
        }
        case PIR_AND_TIMER:
        {
            config_pir.threshold = ((uint32_t) update_config->pir_conf->threshold)*PIR_THRESHOLD_MULTIPLY_FACTOR;
            mcp4012_set_value(update_config->pir_conf->amplification);
            intr_trig_time_in = (uint32_t)update_config->pir_conf->intr_trig_timer;
            timer_interval_in = update_config->timer_conf->timer_interval;
            break;
        }
    }
    data_process_config(update_config, config.signal_out_pin_array);
}

void sensepi_pir_add_tick(uint32_t interval)
{
    log_printf("SensePi Add ticks : %d", interval);
    working_mode = config.config_sensepi->trig_conf;
    switch(working_mode)
    {
        case PIR_ONLY:
        {
            pir_operation(interval);
            break;
        }
        case TIMER_ONLY:
        {
            if(light_check(config.config_sensepi->timer_conf->oper_time) == true
                    && use_timer == false)
            {
                timer_interval_count +=interval;
                if(timer_interval_count >= timer_interval_in)
                {
                    data_process_pattern_gen(TIMER_DATA_PROCESS_MODE);
                }
            }
            break;
        }
        case PIR_AND_TIMER:
        {
            pir_operation(interval);

            if(light_check(config.config_sensepi->timer_conf->oper_time) == true
                    && use_timer == false)
            {
                timer_interval_count +=interval;
                if(timer_interval_count >= timer_interval_in)
                {
                    data_process_pattern_gen(TIMER_DATA_PROCESS_MODE);
                }
            }
            break;
        }
        
    }
}

sensepi_config * sensepi_pir_get_sensepi_config()
{
    log_printf("SensePi_PIR_get_config\n");
    return (config.config_sensepi);
}
