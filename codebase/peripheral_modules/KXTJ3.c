/*
 *  KXTJ3.c : <Write brief>
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

#include <stddef.h>
#include <stdbool.h>

#include "KXTJ3.h"
#include "hal_twim.h"
#include "log.h"
#include "hal_nop_delay.h"


KXTJ3_resolution_t g_resolution;

KXTJ3_range_t g_range;

volatile bool g_is_i2c_running = false;

static uint8_t g_int_src1, g_int_src2;
static KXTJ3_g_data_t g_buff_acce_val;

int32_t signed_conv (int16_t var)
{
    
    int32_t ret_val, buff;
    
    buff = ((var >> 8) & 0x7F);
    buff = buff + 1;
    buff = buff ^ 0xFFFFFF00;
    ret_val = buff;
    return ret_val;
}

void conv_g_value (int16_t xg, int16_t yg, int16_t zg, KXTJ3_g_data_t * p_gdata)
{
    uint16_t l_signedbit;
    int32_t sxg, syg, szg;
//    log_printf("Original : 0x%x, 0x%x, 0x%x\n", xg,yg,zg);
    switch (g_resolution)
    {
        case KXTJ_RES_8Bit :

            l_signedbit = xg & (1 << 15);
            sxg = (l_signedbit) ? ((xg>>8 ) | 0xFFFFFF00) : xg>>8 ;

            l_signedbit = yg & (1 << 15);
            syg = (l_signedbit) ? ((yg>>8) | 0xFFFFFF00) : yg>>8;
     
            l_signedbit = zg & (1 << 15);
            szg = (l_signedbit) ? ((zg>>8) | 0xFFFFFF00) : zg>>8;
     
            switch (g_range) // range/((2^resolution)/2)
            {
                case KXTJ_RNG_2g :
                    p_gdata->xg = (sxg) * (15.6);  
                    p_gdata->yg = (syg) * (15.625);  
                    p_gdata->zg = (szg) * (15.625);  
                    break;
                case KXTJ_RNG_4g : 
                    p_gdata->xg = (xg * (0031.25));  
                    p_gdata->yg = (yg * (0031.25));  
                    p_gdata->zg = (zg * (0031.25));  
                    break;
                case KXTJ_RNG_8g : 
                    p_gdata->xg = (xg * (0062.5));  
                    p_gdata->yg = (yg * (0062.5));  
                    p_gdata->zg = (zg * (0062.5));  
                    break;
                case KXTJ_RNG_16g : 
                    p_gdata->xg = (xg * (0125));  
                    p_gdata->yg = (yg * (0125));  
                    p_gdata->zg = (zg * (012.5));  
                    break;
            }
            break;
        case KXTJ_RES_12Bit :
            l_signedbit = xg & (1 << 15);
            sxg = (l_signedbit) ? ((xg>>4) | 0xFFFFF000) : xg>>4;

            l_signedbit = yg & (1 << 15);
            syg = (l_signedbit) ? ((yg>>4) | 0xFFFFF000) : yg>>4;
     
            l_signedbit = zg & (1 << 15);
            szg = (l_signedbit) ? ((zg>>4) | 0xFFFFF000) : zg>>4;
            switch (g_range) // range/((2^resolution)/2)
            {
                case KXTJ_RNG_2g :
                    p_gdata->xg = (xg) * (000.09765625);  
                    p_gdata->yg = (yg) * (000.09765625);  
                    p_gdata->zg = (zg) * (000.09765625);  
                    break;
                case KXTJ_RNG_4g : 
                    p_gdata->xg = (xg * (000.1953125));  
                    p_gdata->yg = (yg * (000.1953125));  
                    p_gdata->zg = (zg * (000.1953125));  
                    break;
                case KXTJ_RNG_8g : 
                    p_gdata->xg = (xg * (000.390625));  
                    p_gdata->yg = (yg * (000.390625));  
                    p_gdata->zg = (zg * (000.390625));  
                    break;
                case KXTJ_RNG_16g : 
                    p_gdata->xg = (xg * (000.78125));  
                    p_gdata->yg = (yg * (000.78125));  
                    p_gdata->zg = (zg * (000.78125));  
                    break;
            }
            break;
        case KXTJ_RES_14Bit : 
            l_signedbit = xg & (1 << 15);
            sxg = (l_signedbit) ? ((xg>>2) | 0xFFFFC000) : xg>>2;

            l_signedbit = yg & (1 << 15);
            syg = (l_signedbit) ? ((yg>>2) | 0xFFFFC000) : yg>>2;
     
            l_signedbit = zg & (1 << 15);
            szg = (l_signedbit) ? ((zg>>2) | 0xFFFFC000) : zg>>2;
            switch (g_range) // range/((2^resolution)/2)
            {
                case KXTJ_RNG_2g :
                    p_gdata->xg = (xg * (0));  
                    p_gdata->yg = (yg * (0));  
                    p_gdata->zg = (zg * (0));  
                    break;
                case KXTJ_RNG_4g : 
                    p_gdata->xg = (xg * (0));  
                    p_gdata->yg = (yg * (0));  
                    p_gdata->zg = (zg * (0));  
                    break;
                case KXTJ_RNG_8g : 
                    p_gdata->xg = (xg * (000.09765625));  
                    p_gdata->yg = (yg * (000.09765625));  
                    p_gdata->zg = (zg * (000.09765625));  
                    break;
                case KXTJ_RNG_16g : 
                    p_gdata->xg = (xg * (000.1953125));  
                    p_gdata->yg = (yg * (000.1953125));  
                    p_gdata->zg = (zg * (000.1953125));  
                    break;
            }
            break;
    }
}

void write_reg8 (uint8_t reg, uint8_t value)
{
    uint8_t tx_reg[] = {reg, value};
    g_is_i2c_running = false;
    hal_twim_tx (tx_reg, 2);
    while(hal_twim_is_working () != false);
}

void write_reg16 (uint8_t reg, uint16_t value)
{
    uint8_t tx_reg[3];
    tx_reg[0] = reg;
    tx_reg[1] = value & 0x00FF;
    tx_reg[2] = (value & 0xFF00) >> 8;
    
    hal_twim_tx (tx_reg, 3);
    while(hal_twim_is_working () != false);
    
}

uint8_t read_reg8 (uint8_t reg)
{
    uint8_t l_buff[1];
    g_is_i2c_running = false;
    hal_twim_tx_rx (&reg, 1, l_buff, 1);
    while(hal_twim_is_working () != false);
    return l_buff[0];
}

uint16_t read_reg16 (uint8_t reg)
{
//    uint8_t l_arr_buff[2];
    uint16_t l_ret_val;
    g_is_i2c_running = false;
    hal_twim_tx_rx (&reg, 1, (uint8_t *)&l_ret_val, 2);
    while(hal_twim_is_working () != false);
//    l_ret_val = l_arr_buff[0] | (l_arr_buff[1] << 8);
    return l_ret_val;
}

void twim_evt_handler (twim_err_t evt, twim_transfer_t transfer)
{
//    log_printf("%s : %d, %d\n", __func__, evt, transfer);
    g_is_i2c_running = false;
    
}

void (* kxtj3_handler) (KXTJ3_g_data_t g_data);

hal_twim_init_config_t KXTJ3_twi_config = 
{
    .address = KXTJ3_ADDR,
    .irq_priority = APP_IRQ_PRIORITY_MID,
    .evt_handler = twim_evt_handler,
//    .evt_mask = (TWIM_TX_RX_DONE_MSK|TWIM_RX_DONE_MSK|TWIM_TX_DONE_MSK),
    .frequency = HAL_TWI_FREQ_400K,
};

static void l_disable_pc()
{
    uint8_t l_ctl_reg = 0x00;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    l_ctl_reg &= 0x7E; 
    write_reg8 (KXTJ3_CTRL_REG1, l_ctl_reg);
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
}

static void l_enable_pc()
{
    uint8_t l_ctl_reg= 0;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    l_ctl_reg |= KXTJ3_CTRL_REG1_PC; 
    write_reg8 (KXTJ3_CTRL_REG1, l_ctl_reg);
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
}

static void l_disable_wakeup ()
{
    uint8_t l_ctl_reg = 00;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    
    //WUFE disable
    l_ctl_reg &= 0x7C;
    write_reg8 (KXTJ3_CTRL_REG1, l_ctl_reg);
}

static void l_enable_wakeup ()
{
    uint8_t l_ctl_reg = 0;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    
    l_ctl_reg |= KXTJ3_CTRL_REG1_WUFE;
    write_reg8 (KXTJ3_CTRL_REG1, l_ctl_reg);
}

static void l_set_range ()
{
    uint8_t l_ctl_reg = 0;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    l_ctl_reg &= 0x62; // 11100010
    switch (g_range)
    {
        case KXTJ_RNG_2g : 
            l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_2G;
            break;
        case KXTJ_RNG_4g : 
            l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_4G;
            break;
        case KXTJ_RNG_8g : 
            if(g_resolution == KXTJ_RES_14Bit)
            {
                l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_8G_14;
            }
            else
            {
                l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_8G;
            }
            break;
        case KXTJ_RNG_16g : 
            if(g_resolution == KXTJ_RES_14Bit)
            {
                l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_16G_14;
            }
            else
            {
                l_ctl_reg = KXTJ3_CTRL_REG1_GSEL_16G;
            }
            break;
    }
}

static void l_set_resolution ()
{
    uint8_t l_ctl_reg = 0;
    l_ctl_reg = read_reg8 (KXTJ3_CTRL_REG1);
    l_ctl_reg &= 0x3E;
    switch (g_resolution)
    {
        case KXTJ_RES_8Bit : 
            break;
        case KXTJ_RES_12Bit : 
            l_ctl_reg |= KXTJ3_CTRL_REG1_RES;
            break;
        case KXTJ_RES_14Bit : 
            l_ctl_reg |= KXTJ3_CTRL_REG1_RES;
            break;
    }
}

static void l_set_int_src ()
{
    if(g_int_src1)
    {
        write_reg8 (KXTJ3_INT_CTRL_REG1, g_int_src1);
    }
    if(g_int_src2)
    {
        write_reg8 (KXTJ3_INT_CTRL_REG2, g_int_src2);
    }
}

//static void l_ram_reset ()
//{
//    uint8_t l_ctrl_reg;
//    l_ctrl_reg = read_reg8 (KXTJ3_CTRL_REG2);
//    l_ctrl_reg |= KXTJ3_CTRL_REG2_SRST;
//    write_reg8 (KXTJ3_CTRL_REG2, l_ctrl_reg);
//}

void kxtj3_init (KXTJ3_config_t * init_config)
{
    KXTJ3_twi_config.sda = init_config->i2c_sda;
    KXTJ3_twi_config.scl = init_config->i2c_sck;
    
    hal_twim_init (&KXTJ3_twi_config);
    
    g_range = init_config->range;
    
    g_resolution = init_config->resolution;
    
    g_int_src1 = init_config->intr_src1;
    
    g_int_src2 = init_config->intr_src2;
    
    l_disable_pc ();
//    l_ram_reset ();
//    l_enable_pc ();
//    l_disable_pc ();
    l_disable_wakeup ();
    uint8_t l_buff_reg = KXTJ3_DATA_CTRL_REG_OSA_3P125;
    write_reg8 (KXTJ3_DATA_CTRL_REG, l_buff_reg);
    l_buff_reg = KXTJ3_CTRL_REG2_OWUF_3P125;
    write_reg8 (KXTJ3_CTRL_REG2, l_buff_reg);
    l_set_resolution ();
    l_set_range ();
    l_set_int_src ();
    l_enable_pc ();
    
    
    if(init_config->callback_handler != NULL)
    {
        kxtj3_handler = init_config->callback_handler;
    }
}

void kxtj3_set_resolution (KXTJ3_resolution_t resolution)
{
    g_resolution = resolution;
    
    l_disable_pc ();
    l_set_resolution ();
    l_enable_pc ();
}

void kxtj3_set_range (KXTJ3_range_t range)
{
    g_range = range;
    
    l_disable_pc ();
    l_set_range ();
    l_enable_pc ();
}

void kxtj3_set_intr_src (uint8_t src1, uint8_t src2)
{
    g_int_src1 = src1;
    g_int_src2 = src2;
    
    l_disable_pc ();
    l_set_int_src ();
    l_enable_pc ();
}

KXTJ3_g_data_t * kxtj3_get_acce_value ()
{
    
    int16_t xg_int = (int16_t) read_reg16 (KXTJ3_XOUT_L);
    int16_t yg_int = (int16_t) read_reg16 (KXTJ3_YOUT_L);
    int16_t zg_int = (int16_t) read_reg16 (KXTJ3_ZOUT_L);
//    uint16_t xg_int = (int16_t) 64;
//    uint16_t yg_int = (int16_t) 0;
//    uint16_t zg_int = (int16_t) 0;
    
    
    conv_g_value (xg_int, yg_int, zg_int, &g_buff_acce_val);
    
    return &g_buff_acce_val;
}

void kxtj3_start ()
{
    l_enable_pc ();
}

void kxtj3_standby ()
{
    l_disable_pc ();
}

void kxtj3_enable_wakeup ()
{
    l_enable_wakeup ();
}