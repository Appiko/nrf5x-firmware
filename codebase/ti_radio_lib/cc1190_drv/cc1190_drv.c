/******************************************************************************
 *  Filename: cc1190_drv.c
 *
 *  Description: Implementation file for controling the CC1190 device
 *
 *  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/
#include "msp430.h"
#include "hal_spi_rf.h"
#include "cc112x_def.h"

/******************************************************************************
 * @fn         range_extender_rxon
 *
 * @brief      Enable the RX path of the front end module
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
void range_extender_rxon(void) {

	/* configure txon */
	RF_PA_EN_PxOUT &= ~RF_PA_EN_PIN;
	RF_LNA_EN_PxOUT |= RF_LNA_EN_PIN;

	return;
}

/******************************************************************************
 * @fn         range_extender_txon
 *
 * @brief      Enable the TX path of the front end module
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
void range_extender_txon(void) {

	/* configure txon */
	RF_PA_EN_PxOUT |= RF_PA_EN_PIN;
	RF_LNA_EN_PxOUT &= ~RF_LNA_EN_PIN;

	return;
}

/******************************************************************************
 * @fn         range_extender_idle
 *
 * @brief      Idle the front end module
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
void range_extender_idle(void) {

	/* configure idle */
	RF_PA_EN_PxOUT &= ~RF_PA_EN_PIN;
	RF_LNA_EN_PxOUT &= ~RF_LNA_EN_PIN;

	return;
}


/******************************************************************************
 * @fn         range_extender_init
 *
 * @brief      Initialize the front end module
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
void range_extender_init(void) {

	uint8 regs_uint8;

#ifdef USE_CC112X
	/* lower the maximum output of the transciever to 10dBm */
	regs_uint8 = 0x74;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PA_CFG2, &regs_uint8, 1);
#endif

	/* initialize the IO */
	RF_PA_EN_PxDIR |= RF_PA_EN_PIN;
	RF_LNA_EN_PxDIR |= RF_LNA_EN_PIN;

	/* initialize idle */
	range_extender_idle();

	return;
}
