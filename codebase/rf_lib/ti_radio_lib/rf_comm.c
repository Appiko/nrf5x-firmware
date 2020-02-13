/*
 *  rf_comm.c : <Write brief>
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
#include "string.h"


#include "spi_rf_nrf52.h"
#include "rf_comm.h"
#include "cc112x_def.h"
#include "hal_gpio.h"
#include "nrf.h"
#include "log.h"
#include "hal_nop_delay.h"

#ifndef RF_XTAL_FREQ
#define RF_XTAL_FREQ 32000000
#endif

#define MATH_BUFF1 ((uint32_t)(549755813888/RF_XTAL_FREQ))

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

#define GPIOTE_USED0 GPIOTE_CH_USED_RF_COMM_0 
#define GPIOTE_USED1 GPIOTE_CH_USED_RF_COMM_1 
#define GPIOTE_USED2 GPIOTE_CH_USED_RF_COMM_2 
#define GPIOTE_USED3 GPIOTE_CH_USED_RF_COMM_3 

#define RF_LO_DIVIDER          4             /* there is a hardware LO divider CC112x */


const registerSetting_t default_setting[] = 
{
    {PKT_CFG0,          0x20},
    {PKT_CFG1,          0x05},
    {PKT_CFG2,          0x04},
    {PKT_LEN,           0x0F},
    {IOCFG3,            0xB0},
    {IOCFG2,            0x06},
    {IOCFG1,            0x13},
    {IOCFG0,            0x06},
    {MODCFG_DEV_E,      0x02}, //fdev testing

    {SYNC_CFG1,         0x0B},
    {DCFILT_CFG,        0x1C},
    {PREAMBLE_CFG1,     0x18},
    {IQIC,              0xC6},
    {MDMCFG0,           0x05},
    {AGC_REF,           0x20},
    {AGC_CS_THR,        0x19},
    {AGC_CFG1,          0xA9},
    {AGC_CFG0,          0xCF},
    {FIFO_CFG,          0x00},
    {FS_CFG,            0x12},
    {IF_MIX_CFG,        0x00},
    {FS_DIG1,           0x00},
    {FS_DIG0,           0x5F},
    {FS_CAL1,           0x40},
    {FS_CAL0,           0x0E},
    {FS_DIVTWO,         0x03},
    {FS_DSM0,           0x33},
    {FS_DVC0,           0x17},
    {FS_PFD,            0x50},
    {FS_PRE,            0x6E},
    {FS_REG_DIV_CML,    0x14},
    {FS_SPARE,          0xAC},
    {FS_VCO0,           0xB4},
    {XOSC1,             0x03},
    {XOSC2,             0x04},
};

typedef enum
{
    R_IDLE,
    R_RX,
    R_TX,
}radio_state_t;

volatile radio_state_t g_current_state;

static rf_comm_hw_t g_comm_hw;

uint8_t g_marc_sts1;

static uint8_t g_arr_pkt[260];

void (* gp_tx_done) (uint32_t error);
void (* gp_rx_done) (uint32_t error);
void (* gp_tx_failed) (uint32_t error);
void (* gp_rx_failed) (uint32_t error);

uint32_t math_log (uint32_t num, uint32_t base)
{
    return ((num > (base-1))? 1 +  math_log((num/base), base) : 0);
}
int radio_check_status_flag (uint8_t status_bits)
{
	trx16BitRegAccess((RADIO_READ_ACCESS | RADIO_BURST_ACCESS), 0x2F,
                     (0x00FF & MARC_STATUS1), &g_marc_sts1, 1);
    if((status_bits == MARC_NO_FAILURE) && (g_marc_sts1 & 0xFF))
    {
        return 0;
    }
    else if((status_bits & g_marc_sts1) == status_bits)
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
}

void assign_default ()
{
    uint8_t writeByte;
	for(uint32_t i = 0; i < ARRAY_SIZE(default_setting); i++) {

		if(default_setting[i].addr < 0x2F) {
			writeByte = default_setting[i].data;
			trx8BitRegAccess(RADIO_WRITE_ACCESS, default_setting[i].addr, &writeByte, 1);
		} else {
			writeByte = default_setting[i].data;
			trx16BitRegAccess(RADIO_WRITE_ACCESS, 0x2F , (0xFF & default_setting[i].addr),
					&writeByte, 1);
		}
	}
}

uint32_t rf_comm_radio_init (rf_comm_radio_t * p_radio_params, rf_comm_hw_t * p_comm_hw)
{
    //reset
    log_printf("%s\n", __func__);
    memcpy (&g_comm_hw, p_comm_hw, sizeof(rf_comm_hw_t));
    
    if(p_radio_params->rf_tx_done_handler != NULL)
    {
        gp_tx_done = p_radio_params->rf_tx_done_handler;
    }
    
    if(p_radio_params->rf_rx_done_handler != NULL)
    {
        gp_rx_done = p_radio_params->rf_rx_done_handler;
    }

    if(p_radio_params->rf_tx_failed_handler != NULL)
    {
        gp_tx_failed = p_radio_params->rf_tx_failed_handler;
    }
    
    if(p_radio_params->rf_rx_failed_handler != NULL)
    {
        gp_rx_failed = p_radio_params->rf_rx_failed_handler;
    }
//
//    hal_gpio_cfg_output (g_comm_hw.rf_reset_pin, 1);
//    hal_gpio_pin_set (g_comm_hw.rf_reset_pin);
//    hal_nop_delay_ms (1);
//    hal_gpio_pin_clear (g_comm_hw.rf_reset_pin);
//    hal_nop_delay_ms (1);
//    hal_gpio_pin_set (g_comm_hw.rf_reset_pin);
    //Set default config
	trxSpiCmdStrobe(SRES);

	/* give the tranciever time enough to complete reset cycle */
	hal_nop_delay_us (16000);

    assign_default ();
    rf_comm_set_bw (p_radio_params->rx_bandwidth);
    rf_comm_set_bitrate (p_radio_params->bitrate);
    rf_comm_set_fdev (p_radio_params->freq_dev);

    rf_comm_set_freq (p_radio_params->center_freq);
    rf_comm_set_pwr (p_radio_params->tx_power);
    
//    hal_gpio_cfg_input (g_comm_hw.rf_gpio0_pin, HAL_GPIO_PULL_DISABLED);
//    hal_gpio_cfg_input (g_comm_hw.rf_gpio1_pin, HAL_GPIO_PULL_DISABLED);
    hal_gpio_cfg_input (g_comm_hw.rf_gpio2_pin, HAL_GPIO_PULL_DISABLED);
//    hal_gpio_cfg_input (g_comm_hw.rf_gpio3_pin, HAL_GPIO_PULL_DISABLED);
  
#ifdef RF_COMM_AMPLIFIRE
    hal_gpio_cfg_output (g_comm_hw.rf_hgm_pin, 0);
    hal_gpio_cfg_output (g_comm_hw.rf_pa_pin, 0);
    hal_gpio_cfg_output (g_comm_hw.rf_lna_pin, 0);
    
#endif
        
    NRF_GPIOTE->CONFIG[GPIOTE_USED0] = 
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
        | (g_comm_hw.rf_gpio2_pin << GPIOTE_CONFIG_PSEL_Pos)
        | (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
    NRF_GPIOTE->INTENSET = 1 << GPIOTE_USED0;

    NVIC_SetPriority (GPIOTE_IRQn, p_radio_params->irq_priority);
    NVIC_EnableIRQ (GPIOTE_IRQn);

    return 0;
}

uint32_t rf_comm_set_freq (uint32_t freq)
{
    freq = freq*1000;
	uint8_t freq_regs[3];
	uint32_t freq_regs_uint32;
	uint32_t f_vco;
    const uint32_t math_buff1 = RF_XTAL_FREQ/ (1 << 16);

	/* Radio frequency -> VCO frequency */
	f_vco = freq * RF_LO_DIVIDER;

	/* Divide by oscillator frequency */

	/* Convert value into uint32_t from float */
	freq_regs_uint32 = (f_vco/ math_buff1);

	/* return the frequency word */

    freq_regs[0] = (freq_regs_uint32 >> 16)&0xFF;
    freq_regs[1] = (freq_regs_uint32 >> 8)&0xFF;
    freq_regs[2] = (freq_regs_uint32 & 0xFF);

	/* write the frequency word to the transciever */
	trx16BitRegAccess(RADIO_WRITE_ACCESS | RADIO_BURST_ACCESS, 0x2F, (0xFF & FREQ2), freq_regs, 3);
//    log_printf("%s : 0x%x\n", __func__,freq_regs_uint32);

    return 0;

}

uint32_t rf_comm_set_bitrate (uint32_t bitrate)
{
    uint8_t srate_e;
    uint32_t srate_m;
    uint8_t arr_reg[3];
    
    
    srate_e = math_log((bitrate*MATH_BUFF1),2) - 20;
    
    srate_m = (bitrate*MATH_BUFF1/ (1 << srate_e)) - (1 << 20);
    if(srate_m < (1 << 8))
    {
        arr_reg[0] = srate_m & 0xFF;
        trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & SYMBOL_RATE0), arr_reg, 1);
        arr_reg[0] = (srate_e<<4) & 0xF0;
        trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & SYMBOL_RATE2), arr_reg, 1);
//        log_printf("%s : %d %d (8bit)\n", __func__, srate_m, srate_e);
    }
    else if(srate_m < (1 << 16))
    {
        arr_reg[0] = (srate_m & 0xFF00) >> 8;
        arr_reg[1] = (srate_m & 0xFF);
        trx8BitRegAccess (RADIO_WRITE_ACCESS | RADIO_BURST_ACCESS, (0xFF & SYMBOL_RATE1), arr_reg, 2);
        arr_reg[0] = (srate_e << 4) & 0xF0;
        trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & SYMBOL_RATE2), arr_reg, 1);
//        log_printf("%s : %d %d (16bit)\n", __func__, srate_m, srate_e);
    }
    else if (srate_m == (1<< 20))
    {
        arr_reg[0] = 0;
        trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & SYMBOL_RATE0), arr_reg, 1);
        arr_reg[0] = ((srate_e + 1) << 4)& 0xF0;
        trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & SYMBOL_RATE2), arr_reg, 1);
//        log_printf("%s : %d %d (20bit-0bit)\n", __func__, srate_m, srate_e);        
    }
    else 
    {
        arr_reg[0] = ((srate_m & 0x0F0000) >> 16) | ((srate_e << 4)& 0xF0);
        arr_reg[1] = (srate_m & 0xFF00) >> 8;
        arr_reg[2] = (srate_m & 0xFF);
        trx8BitRegAccess (RADIO_WRITE_ACCESS | RADIO_BURST_ACCESS,
                          (0xFF & SYMBOL_RATE2), arr_reg, 3);
//        log_printf("%s : %d %d (20bit)\n", __func__, srate_m, srate_e);
    }
//    trx8BitRegAccess (RADIO_READ_ACCESS | RADIO_BURST_ACCESS, (0xFF & SYMBOL_RATE2), arr_reg, 3);
//    log_printf(" %d %d\n", (arr_reg[0]&0xF0)>>4,
//               ((arr_reg[0]&0x0F) << 16) | (arr_reg[1] << 8) | arr_reg[2]);
    return 1;
    
}

uint32_t rf_comm_set_fdev (uint32_t fdev)
{
    if(fdev / 1000 == 0)
    {
        fdev = fdev * 1000;
    }
    uint32_t manti = 256;
    uint8_t exp, reg;
    const uint32_t l_math_buff0 =  ROUNDED_DIV((1 << 24), 32000000);
    exp = 0x00;
    while(manti > 255)
    {
        exp++;
        manti = (uint32_t)(ROUNDED_DIV(fdev, (1 << exp)) / l_math_buff0)
            - 256;

    }
//	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & MARCSTATE, &reg_status, 1);

//    exp--;
    reg = 0x00;
    reg |= (0x08);
    reg |= (0x07 & exp);
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & MODCFG_DEV_E), &reg, 1);
    
    
    reg = (uint8_t)(0xFF & manti);
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & DEVIATION_M), &reg, 1);
//    log_printf("%s : %d %d\n", __func__, manti, exp);
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & DEVIATION_M), &reg, 1);
    uint8_t arr_reg[2];
    trx8BitRegAccess (RADIO_READ_ACCESS|RADIO_BURST_ACCESS,
                      (0xFF & DEVIATION_M), arr_reg, 2);
    
//    log_printf("0x%x 0x%x\n", arr_reg[0], arr_reg[1]);
    return 0;
}

uint32_t rf_comm_set_bw (uint32_t bandwidth)
{
    //Bypass will be always enabled and Decimation factor will always be 20
    //refer "RX filter bandwidth" section in CC112x datasheet.
    if((int)(bandwidth/1000) == 0)
    {
        bandwidth = bandwidth*1000;
    }
    uint8_t bb_cic_decfact = 1;
    
    bb_cic_decfact = RF_XTAL_FREQ / (20 * 8 * bandwidth);
    
    bb_cic_decfact = bb_cic_decfact & 0x3F;
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & CHAN_BW), &bb_cic_decfact, 1);
    log_printf("%s : %d\n",__func__, bb_cic_decfact);
//    trx8BitRegAccess (RADIO_READ_ACCESS, (0xFF & CHAN_BW), &bb_cic_decfact, 1);
//    log_printf("%d\n", bb_cic_decfact);
    return 0;
    
}

uint32_t rf_comm_set_pwr (int32_t pwr)
{
    //For now it is assumed bitrate < 5kbps
    if (pwr > 14)
    {
        pwr = 14;
    }
    //ToDo : calculations for upsampler
    uint8_t reg = 0x7E;
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & PA_CFG0), &reg, 1);
    reg = 0x56;
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & PA_CFG1), &reg, 1);
    
    reg = 2*(pwr+18) - 1;
    reg |= 0x40;
    
    trx8BitRegAccess (RADIO_WRITE_ACCESS, (0xFF & PA_CFG2), &reg, 1);
    log_printf("%s : %d\n", __func__, reg);
    uint8_t arr_reg[3];
    trx8BitRegAccess (RADIO_READ_ACCESS|RADIO_BURST_ACCESS, (0xFF & PA_CFG2), arr_reg, 3);
    log_printf("0x%x 0x%x 0x%x\n", arr_reg[0],arr_reg[1],arr_reg[2]);
    return 0;
    
    
}

uint32_t rf_comm_pkt_config (rf_comm_pkt_t * p_pkt_config)
{ 
    g_arr_pkt[0] = p_pkt_config->max_len;  //Change this values
    g_arr_pkt[1] = p_pkt_config->app_id;
    g_arr_pkt[2] = (p_pkt_config->dev_id & 0xFF00)>>8;
    g_arr_pkt[3] = p_pkt_config->dev_id & 0xFF;
    return 0;
}

uint32_t rf_comm_pkt_send (uint8_t pkt_type, uint8_t * p_data, uint8_t len)
{
    trxSpiCmdStrobe (SFTX);
#ifdef RF_COMM_AMPLIFIRE
    hal_gpio_pin_set (g_comm_hw.rf_hgm_pin);
    hal_gpio_pin_set (g_comm_hw.rf_pa_pin);
#endif
    g_arr_pkt[0] = 4+len;  //Change this values
    g_arr_pkt[4] = pkt_type;
    memcpy (&g_arr_pkt[5], p_data, len);
    
	trx8BitRegAccess(RADIO_WRITE_ACCESS|RADIO_BURST_ACCESS, TXFIFO, g_arr_pkt, len+5);
	trxSpiCmdStrobe(STX);               // Change state to TX, initiating
    g_current_state = R_TX;
    return 0;
}

uint32_t rf_comm_rx_enable ()
{
    g_current_state = R_RX;
    trxSpiCmdStrobe (SFRX);
	trxSpiCmdStrobe(SRX);               // Change state to RX, initiating
    return 0;
}


uint32_t rf_comm_pkt_receive (uint8_t * p_rxbuff, uint8_t * p_len)
{
    uint8_t pktLen;
    uint8_t status;
#ifdef RF_COMM_AMPLIFIRE
    hal_gpio_pin_set (g_comm_hw.rf_lna_pin);
    hal_gpio_pin_set (g_comm_hw.rf_hgm_pin);
#endif
//	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & NUM_RXBYTES, &pktLen, 1);

    trx8BitRegAccess(RADIO_READ_ACCESS, RXFIFO, &pktLen, 1);
    *p_len = pktLen;
    log_printf("Pkt Len : %d\n", pktLen);
	if (pktLen > 0)
    {

		trx8BitRegAccess(RADIO_READ_ACCESS|RADIO_BURST_ACCESS, RXFIFO, p_rxbuff, pktLen);
		/* retrieve the FIFO content */


		/* retrieve the CRC status information */
		trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & LQI_VAL, &status, 1);

		/* Return CRC_OK bit */
		status  = status & CRC_OK;
        trxSpiCmdStrobe(SFRX);	                     // Flush RXFIFO

	}
    g_current_state = R_RX;
    return status;
}

uint32_t rf_comm_idle ()
{
	/* Force transciever idle state */
	trxSpiCmdStrobe(SIDLE);

	/* Flush the FIFO's */
	trxSpiCmdStrobe(SFRX);
	trxSpiCmdStrobe(SFTX);

	return(0);
}

uint32_t rf_comm_sleep ()
{
#ifdef RF_COMM_AMPLIFIRE
    hal_gpio_pin_clear (g_comm_hw.rf_hgm_pin);
    hal_gpio_pin_clear (g_comm_hw.rf_lna_pin);
    hal_gpio_pin_clear (g_comm_hw.rf_pa_pin);
#endif
	/* Force transciever idle state */
    trxSpiCmdStrobe(SRES);
    hal_nop_delay_ms (20);

    while(rf_comm_get_state ());

	/* Enter sleep state on exit */
	trxSpiCmdStrobe(SPWD);

	return(0);
}


uint32_t rf_comm_flush(void)
{
	trxSpiCmdStrobe(SFRX);
	trxSpiCmdStrobe(SFTX);
	return(0);
}

uint32_t rf_comm_wake(void)
{

	/* Force transciever idle state */
	trxSpiCmdStrobe(SIDLE);

	/* 1 ms delay for letting RX settle */
	hal_nop_delay_us (5000);

	return(0);
}

int8_t rf_comm_get_rssi ()
{
	int8_t rssi;
	uint8_t cc_rssi[2];

	trx16BitRegAccess(RADIO_READ_ACCESS , 0x2F, (0xFF & RSSI1), cc_rssi, 2);
    
    
    if(cc_rssi[1] & RSSI0_RSSI_VALID)
    {

        rssi = cc_rssi[0];
        if (rssi >= 128) {
            rssi = rssi - 256;
        }
        rssi = rssi - 99;
        return rssi;
    }
    else
    {
        return 0xFF;
    }


}

void rf_comm_disable_irq ()
{
        
    NRF_GPIOTE->CONFIG[GPIOTE_USED0] = 0;
    NRF_GPIOTE->INTENCLR = 1 << GPIOTE_USED0;
    
}

void rf_comm_enable_irq()
{
        
    NRF_GPIOTE->CONFIG[GPIOTE_USED0] = 
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
        | (g_comm_hw.rf_gpio2_pin << GPIOTE_CONFIG_PSEL_Pos)
        | (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
    NRF_GPIOTE->INTENSET = 1 << GPIOTE_USED0;

}


uint32_t rf_comm_get_state ()
{
    uint8_t state;
	state = (uint8_t)trxSpiCmdStrobe (SNOP);
    state &= 0xF0;
    state >>= 4;
    return state;
}
uint32_t rf_comm_get_radio_id ()
{
	uint8_t ret_partnum;
	trx16BitRegAccess(RADIO_READ_ACCESS , 0x2F, (0xFF & PARTNUMBER), &ret_partnum, 1);
    return ret_partnum;
}

#if ISR_MANAGER == 1
void rf_comm_gpiote_Handler ()
#else
void GPIOTE_IRQHandler ()
#endif
{
    if(NRF_GPIOTE->EVENTS_IN[GPIOTE_USED0])
    {
#if ISR_MANAGER == 0
        NRF_GPIOTE->EVENTS_IN[GPIOTE_USED0] = 0;
#endif
        
//        if(radio_check_status_flag (MARC_NO_FAILURE)) 
//        {
//            if(g_current_state == R_TX)
//            {
//                log_printf("Tx Done\n");
//                g_current_state = R_IDLE;
//                if(gp_tx_done != NULL)
//                {
//                    gp_tx_done (g_marc_sts1);
//                }
//            }
//            if(g_current_state == R_RX)
//            {
////                log_printf("Rx Done\n");
//                g_current_state = R_IDLE;
//                if(gp_rx_done != NULL)
//                {
//                    gp_rx_done (g_marc_sts1);
//                }
//            }
//        }
        log_printf("%s : %d %d\n", __func__, g_current_state, g_marc_sts1);
    	trx16BitRegAccess((RADIO_READ_ACCESS | RADIO_BURST_ACCESS), 0x2F,
                     (0x00FF & MARC_STATUS1), &g_marc_sts1, 1);
        
        if((g_marc_sts1 & MARC_TX_SUCCESSFUL) == MARC_TX_SUCCESSFUL) 
        {
            if(g_current_state == R_TX)
            {
                log_printf("Tx Done\n");
                g_current_state = R_IDLE;
                if(gp_tx_done != NULL)
                {
                    gp_tx_done (g_marc_sts1);
                }
            }
        }
        else if((g_marc_sts1 & MARC_RX_SUCCESSFUL) == MARC_RX_SUCCESSFUL) 
        {
            if(g_current_state == R_RX)
            {
                log_printf("Rx Done\n");
                g_current_state = R_IDLE;
                if(gp_rx_done != NULL)
                {
                    gp_rx_done (g_marc_sts1);
                }
            }
        }

        else
        {
            if(g_current_state == R_TX)
            {
                log_printf("Tx Failed\n");
                g_current_state = R_IDLE;
                if(gp_tx_failed != NULL)
                {
                    gp_tx_failed (g_marc_sts1);
                }
            }
            if(g_current_state == R_RX)
            {
                log_printf("Rx Failed\n");
                g_current_state = R_IDLE;
                if(gp_rx_failed != NULL)
                {
                    gp_rx_failed (g_marc_sts1);
                }
            }
        }
    }
}
