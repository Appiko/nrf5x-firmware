/**
 *  opt3101_oper.c : OPT3101 basic operations driver.
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "opt3101_oper.h"
#include "opt3101_reg.h"

uint8_t opt3101_oper_init ()
{
    uint8_t status = 0;
    opt3101_reg_modify(FORCE_EN_SLAVE, FORCE_EN_SLAVE_LSB, FORCE_EN_SLAVE_MSB, 1);  //Since I2C Master bus is floating this register needs to be set to enable device to respond

    opt3101_reg_modify(TG_OVL_WINDOW_START, TG_OVL_WINDOW_START_LSB,
                     TG_OVL_WINDOW_START_MSB, 7000); // //Overload flab observation window
    opt3101_reg_modify(EN_TEMP_CONV, EN_TEMP_CONV_LSB, EN_TEMP_CONV_MSB, 1); // //Enables the internal

    opt3101_reg_modify(CLIP_MODE_FC, CLIP_MODE_FC_LSB, CLIP_MODE_FC_MSB, 1); // //Enables Clip mode for Frequency correction
    opt3101_reg_modify(CLIP_MODE_TEMP, CLIP_MODE_TEMP_LSB, CLIP_MODE_TEMP_MSB, 0); // //Disables Clip mode for Temp coff phase correction
    opt3101_reg_modify(CLIP_MODE_OFFSET, CLIP_MODE_OFFSET_LSB, CLIP_MODE_OFFSET_MSB, 0); // //Disables Clip mode for phase offset correction
    opt3101_reg_modify(IQ_READ_DATA_SEL, IQ_READ_DATA_SEL_LSB, IQ_READ_DATA_SEL_MSB, 3); // //Enables 16 bit frame counter
    opt3101_reg_modify(IAMB_MAX_SEL, IAMB_MAX_SEL_LSB, IAMB_MAX_SEL_MSB, 0); // //Sets maximum ambient support
    opt3101_reg_modify(EN_TEMP_CORR, EN_TEMP_CORR_LSB, EN_TEMP_CORR_MSB, 1); // //Enables Temperature Correction
    opt3101_reg_modify(GPIO1_OBUF_EN, GPIO1_OBUF_EN_LSB, GPIO1_OBUF_EN_MSB, 1); // //Enabled output buffer on GPIO1 pin
    opt3101_reg_modify(GPO1_MUX_SEL, GPO1_MUX_SEL_LSB, GPO1_MUX_SEL_MSB, 2); 	    // //select dig_gpo_0 on gpio1
    opt3101_reg_modify(DIG_GPO_SEL0, DIG_GPO_SEL0_LSB, DIG_GPO_SEL0_MSB, 9); 	// //Select Data Ready on dig_gpo_0

    opt3101_reg_modify(NUM_SUB_FRAMES, NUM_SUB_FRAMES_LSB, NUM_SUB_FRAMES_MSB, 3); // //Sub frames count
    opt3101_reg_modify(NUM_AVG_SUB_FRAMES, NUM_AVG_SUB_FRAMES_LSB, NUM_AVG_SUB_FRAMES_MSB, 1); // //Average frames count
    opt3101_reg_modify(XTALK_FILT_TIME_CONST, XTALK_FILT_TIME_CONST_LSB, XTALK_FILT_TIME_CONST_MSB, 9); // //Crosstalk filter time constant
    opt3101_reg_modify(TG_SEQ_INT_START, TG_SEQ_INT_START_LSB, TG_SEQ_INT_START_MSB, 9850); // //Sequence Start
    opt3101_reg_modify(TG_SEQ_INT_END, TG_SEQ_INT_END_LSB, TG_SEQ_INT_END_MSB, 9858); // //Sequence End
    opt3101_reg_modify(TG_SEQ_INT_MASK_START, TG_SEQ_INT_MASK_START_LSB, TG_SEQ_INT_MASK_START_MSB, 1); // //Same as AvgFrame Count
    opt3101_reg_modify(TG_SEQ_INT_MASK_END, TG_SEQ_INT_MASK_END_LSB, TG_SEQ_INT_MASK_END_MSB, 1); // //Same as AvgFrame Count

    opt3101_reg_modify(HDR_THR_HIGH, HDR_THR_HIGH_LSB, HDR_THR_HIGH_MSB, 25520); // //High Threshold
    opt3101_reg_modify(HDR_THR_LOW, HDR_THR_LOW_LSB, HDR_THR_LOW_MSB, 5880); // //Low Threshold
    opt3101_reg_modify(EN_ADAPTIVE_HDR, EN_ADAPTIVE_HDR_LSB, EN_ADAPTIVE_HDR_MSB, 1); // //Enables adaptive HDR feature

    opt3101_reg_modify(ILLUM_DAC_H_TX0, ILLUM_DAC_H_TX0_LSB, ILLUM_DAC_H_TX0_MSB, 31); // //High Current settings [173.6mA:5.6mA X 31]
    opt3101_reg_modify(ILLUM_SCALE_H_TX0, ILLUM_SCALE_H_TX0_LSB, ILLUM_SCALE_H_TX0_MSB, 0); // //Illum scale for H [173.6mA:5.6mA X 31]

    opt3101_reg_modify(ILLUM_DAC_L_TX0, ILLUM_DAC_L_TX0_LSB, ILLUM_DAC_L_TX0_MSB, 31); // //High Current settings [043.4mA:1.4mA X 31]
    opt3101_reg_modify(ILLUM_SCALE_L_TX0, ILLUM_SCALE_L_TX0_LSB, ILLUM_SCALE_L_TX0_MSB, 3); // //Illum scale for H [043.4mA:1.4mA X 31]

    opt3101_reg_modify(TX_SEQ_REG, TX_SEQ_REG_LSB, TX_SEQ_REG_MSB, 2184); // //Setting TX Switching order
    opt3101_reg_modify(EN_TX_SWITCH, EN_TX_SWITCH_LSB, EN_TX_SWITCH_MSB, 1); // //Enable TX Switching order

    opt3101_reg_modify(TG_EN, TG_EN_LSB, TG_EN_MSB, 1); // //Enables Timing Generator


    return status;
}

uint8_t opt3101_oper_start ()
{
    uint8_t status = 0;
    opt3101_reg_modify (MONOSHOT_BIT, MONOSHOT_BIT_LSB, MONOSHOT_BIT_MSB, 1);
    return status;
    
}
uint8_t opt3101_oper_sel_mode (opt3101_oper_mode_t mode)
{
    uint8_t status = 0;
    if(mode == OPT3101_OPER_MONOSHOT)
    {
        opt3101_reg_modify (MONOSHOT_MODE, MONOSHOT_MODE_LSB, MONOSHOT_MODE_MSB, 3);
    }
    else
    {
        opt3101_reg_modify (MONOSHOT_MODE, MONOSHOT_MODE_LSB, MONOSHOT_MODE_MSB, 0);
        
    }
    return status;
}

uint32_t opt3101_oper_get_dist ()
{
    uint32_t phase_out;
    opt3101_reg_read (PHASE_OUT, PHASE_OUT_LSB, PHASE_OUT_MSB, &phase_out);
    return (uint32_t)((phase_out * 228.72) / 1000);
}
