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
    opt3101_reg_modify(FORCE_EN_SLAVE, FORCE_EN_SLAVE_LSB, FORCE_EN_SLAVE_MSB, 1); // //Since I2C Master bus is floating this register needs to be set to enable device to respond

    opt3101_reg_modify(TG_OVL_WINDOW_START, TG_OVL_WINDOW_START_LSB,
                       TG_OVL_WINDOW_START_MSB, 7000); // //Overload flab observation window
    opt3101_reg_modify(EN_TEMP_CONV, EN_TEMP_CONV_LSB, EN_TEMP_CONV_MSB, 1); // //Enables the internal

    opt3101_reg_modify(CLIP_MODE_FC, CLIP_MODE_FC_LSB, CLIP_MODE_FC_MSB, 1); // //Enables Clip mode for Frequency correction
    opt3101_reg_modify(CLIP_MODE_TEMP, CLIP_MODE_TEMP_LSB, CLIP_MODE_TEMP_MSB, 0); // //Disables Clip mode for Temp coff phase correction
    opt3101_reg_modify(CLIP_MODE_OFFSET, CLIP_MODE_OFFSET_LSB, CLIP_MODE_OFFSET_MSB, 0); // //Disables Clip mode for phase offset correction
    opt3101_reg_modify(IQ_READ_DATA_SEL, IQ_READ_DATA_SEL_LSB, IQ_READ_DATA_SEL_MSB, 3); // //Enables 16 bit frame counter
    opt3101_reg_modify(IAMB_MAX_SEL, IAMB_MAX_SEL_LSB, IAMB_MAX_SEL_MSB, 11); // //Sets maximum ambient support
    opt3101_reg_modify(EN_TEMP_CORR, EN_TEMP_CORR_LSB, EN_TEMP_CORR_MSB, 1); // //Enables Temperature Correction
    opt3101_reg_modify(GPIO1_OBUF_EN, GPIO1_OBUF_EN_LSB, GPIO1_OBUF_EN_MSB, 1); // //Enabled output buffer on GPIO1 pin
    opt3101_reg_modify(GPO1_MUX_SEL, GPO1_MUX_SEL_LSB, GPO1_MUX_SEL_MSB, 2); 	    // //select dig_gpo_0 on gpio1
    opt3101_reg_modify(DIG_GPO_SEL0, DIG_GPO_SEL0_LSB, DIG_GPO_SEL0_MSB, 9); 	// //Select Data Ready on dig_gpo_0

    opt3101_reg_modify(EN_CONT_FCALIB, EN_CONT_FCALIB_LSB, EN_CONT_FCALIB_MSB, 1); // //Enables continuous frequency calibration
    opt3101_reg_modify(START_FREQ_CALIB, START_FREQ_CALIB_LSB, START_FREQ_CALIB_MSB, 1); // //Starts the frequency calibration block
    opt3101_reg_modify(EN_FLOOP, EN_FLOOP_LSB, EN_FLOOP_MSB, 1); // //Enables the frequency correction loop
    opt3101_reg_modify(EN_AUTO_FREQ_COUNT, EN_AUTO_FREQ_COUNT_LSB, EN_AUTO_FREQ_COUNT_MSB, 1); // //Enables automatic frequency count
    opt3101_reg_modify(EN_FREQ_CORR, EN_FREQ_CORR_LSB, EN_FREQ_CORR_MSB, 1); // //Enables digital frequency correction
    opt3101_reg_modify(SYS_CLK_DIVIDER, SYS_CLK_DIVIDER_LSB, SYS_CLK_DIVIDER_MSB, 10);  // //Divider for system clock
    opt3101_reg_modify(REF_COUNT_LIMIT, REF_COUNT_LIMIT_LSB, REF_COUNT_LIMIT_MSB, 19531); // //Counter limit
    opt3101_reg_modify(GPIO2_OBUF_EN, GPIO2_OBUF_EN_LSB, GPIO2_OBUF_EN_MSB, 0); // //Disables output buffer on GPIO2
    opt3101_reg_modify(GPIO2_IBUF_EN, GPIO2_IBUF_EN_LSB, GPIO2_IBUF_EN_MSB, 1); // //Enables ref clock input of GPIO2

    opt3101_reg_modify(NUM_SUB_FRAMES, NUM_SUB_FRAMES_LSB, NUM_SUB_FRAMES_MSB, 7); // //Sub frames count
    opt3101_reg_modify(NUM_AVG_SUB_FRAMES, NUM_AVG_SUB_FRAMES_LSB, NUM_AVG_SUB_FRAMES_MSB, 7); // //Average frames count
    opt3101_reg_modify(XTALK_FILT_TIME_CONST, XTALK_FILT_TIME_CONST_LSB, XTALK_FILT_TIME_CONST_MSB, 7); // //Crosstalk filter time constant
    opt3101_reg_modify(TG_SEQ_INT_START, TG_SEQ_INT_START_LSB, TG_SEQ_INT_START_MSB, 9850); // //Sequence Start
    opt3101_reg_modify(TG_SEQ_INT_END, TG_SEQ_INT_END_LSB, TG_SEQ_INT_END_MSB, 9858); // //Sequence End
    opt3101_reg_modify(TG_SEQ_INT_MASK_START, TG_SEQ_INT_MASK_START_LSB, TG_SEQ_INT_MASK_START_MSB, 7); // //Same as AvgFrame Count
    opt3101_reg_modify(TG_SEQ_INT_MASK_END, TG_SEQ_INT_MASK_END_LSB, TG_SEQ_INT_MASK_END_MSB, 7); // //Same as AvgFrame Count

    opt3101_reg_modify(COMMAND0, COMMAND0_LSB, COMMAND0_MSB, 0x108); // //Set Channel 1
    opt3101_reg_modify(COMMAND1, COMMAND1_LSB, COMMAND1_MSB, 0xB02); // //COMP1.
    opt3101_reg_modify(COMMAND2, COMMAND2_LSB, COMMAND2_MSB, 0x100); // //Set Channel 0
    opt3101_reg_modify(COMMAND3, COMMAND3_LSB, COMMAND3_MSB, 0xC00); // //COMP1_INV
    opt3101_reg_modify(COMPARE_REG1, COMPARE_REG1_LSB, COMPARE_REG1_MSB, 26000); // //ThresholdH
    opt3101_reg_modify(COMPARE_REG2, COMPARE_REG2_LSB, COMPARE_REG2_MSB, 4312); // //ThresholdH
    opt3101_reg_modify(MUX_SEL_COMPIN, MUX_SEL_COMPIN_LSB, MUX_SEL_COMPIN_MSB, 0); // //Selects Amplitude for Comparison
    opt3101_reg_modify(EN_TX1_ON_TX0, EN_TX1_ON_TX0_LSB, EN_TX1_ON_TX0_MSB, 1); // //Setting TX1 register and connect to TX0

    opt3101_reg_modify(EN_PROCESSOR_VALUES, EN_PROCESSOR_VALUES_LSB, EN_PROCESSOR_VALUES_MSB, 1); // //Enables processor values
    opt3101_reg_modify(EN_SEQUENCER, EN_SEQUENCER_LSB, EN_SEQUENCER_MSB, 1); // //Enables the Sequencer
    opt3101_reg_modify(HDR_THR_HIGH, HDR_THR_HIGH_LSB, HDR_THR_HIGH_MSB, 25500); // //High Threshold
    opt3101_reg_modify(HDR_THR_LOW, HDR_THR_LOW_LSB, HDR_THR_LOW_MSB, 4812); // //Low Threshold
    opt3101_reg_modify(EN_ADAPTIVE_HDR, EN_ADAPTIVE_HDR_LSB, EN_ADAPTIVE_HDR_MSB, 1); // //Enables adaptive HDR feature

    opt3101_reg_modify(ILLUM_DAC_H_TX0, ILLUM_DAC_H_TX0_LSB, ILLUM_DAC_H_TX0_MSB, 2); // //High Current settings [011.2mA:5.6mA X 02]
    opt3101_reg_modify(ILLUM_SCALE_H_TX0, ILLUM_SCALE_H_TX0_LSB, ILLUM_SCALE_H_TX0_MSB, 0); // //Illum scale for H [011.2mA:5.6mA X 02]

    opt3101_reg_modify(ILLUM_DAC_L_TX0, ILLUM_DAC_L_TX0_LSB, ILLUM_DAC_L_TX0_MSB, 1); // //High Current settings [002.8mA:2.8mA X 01]
    opt3101_reg_modify(ILLUM_SCALE_L_TX0, ILLUM_SCALE_L_TX0_LSB, ILLUM_SCALE_L_TX0_MSB, 2); // //Illum scale for H [002.8mA:2.8mA X 01]

    opt3101_reg_modify(ILLUM_DAC_H_TX1, ILLUM_DAC_H_TX1_LSB, ILLUM_DAC_H_TX1_MSB, 31); // //High Current settings [173.6mA:5.6mA X 31]
    opt3101_reg_modify(ILLUM_SCALE_H_TX1, ILLUM_SCALE_H_TX1_LSB, ILLUM_SCALE_H_TX1_MSB, 0); // //Illum scale for H [173.6mA:5.6mA X 31]

    opt3101_reg_modify(ILLUM_DAC_L_TX1, ILLUM_DAC_L_TX1_LSB, ILLUM_DAC_L_TX1_MSB, 31); // //High Current settings [043.4mA:1.4mA X 31]
    opt3101_reg_modify(ILLUM_SCALE_L_TX1, ILLUM_SCALE_L_TX1_LSB, ILLUM_SCALE_L_TX1_MSB, 3); // //Illum scale for H [043.4mA:1.4mA X 31]

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
