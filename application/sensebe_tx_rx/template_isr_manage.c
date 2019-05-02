/**
 *  template_isr_manage.c : Template for ISR sharing.
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

#include "template_isr_manage.h"
#include "nrf.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

void POWER_CLOCK_IRQHandler (void)
{
}

void RADIO_IRQHandler (void)
{
#if defined HAL_RADIO_PERIPH_USED
    hal_radio_Handler ();
#endif

#if defined RADIO_PERIPH_USED_BLE_ADV
    ble_adv_radio_Handler ();
#endif
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_BCMATCH = 0;
    NRF_RADIO->EVENTS_CRCERROR = 0;
    NRF_RADIO->EVENTS_CRCOK = 0;
    NRF_RADIO->EVENTS_DEVMATCH = 0;
    NRF_RADIO->EVENTS_DEVMISS = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_RSSIEND = 0;
    
}

void UARTE0_UART0_IRQHandler (void)
{
#if defined LOG_UART_PRINTF
    hal_uart_Handler ();
#elif defined LOG_UART_DMA_PRINTF
    uart_printf_uart_Handler ();
#endif
//Clear events
    NRF_UARTE0->EVENTS_CTS = 0;
    NRF_UARTE0->EVENTS_ENDRX = 0;
    NRF_UARTE0->EVENTS_ENDTX = 0;
    NRF_UARTE0->EVENTS_ERROR = 0;
    NRF_UARTE0->EVENTS_NCTS = 0;
    NRF_UARTE0->EVENTS_RXDRDY = 0;
    NRF_UARTE0->EVENTS_RXSTARTED = 0;
    NRF_UARTE0->EVENTS_RXTO = 0;
    NRF_UARTE0->EVENTS_TXDRDY = 0;
    NRF_UARTE0->EVENTS_TXSTARTED = 0;
    NRF_UARTE0->EVENTS_TXSTOPPED = 0;
    
}

void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler (void)
{
#if defined HAL_SPIM_PERIPH_USED
#if HAL_SPIM_PERIPH_USED == 0
    hal_spim_Handler ();
#endif
#endif
#if defined HAL_TWIM_PERIPH_USED 
#if HAL_TWIM_PERIPH_USED == 0
    hal_twim_Handler ();
#endif
#endif
//Clear events
    NRF_SPIM0->EVENTS_END = 0;
    NRF_SPIM0->EVENTS_ENDRX = 0;
    NRF_SPIM0->EVENTS_ENDTX = 0;
    NRF_SPIM0->EVENTS_STARTED = 0;
    NRF_SPIM0->EVENTS_STOPPED = 0;
    
    NRF_TWIM0->EVENTS_ERROR = 0;
    NRF_TWIM0->EVENTS_LASTRX = 0;
    NRF_TWIM0->EVENTS_LASTTX = 0;
    NRF_TWIM0->EVENTS_RXSTARTED = 0;
    NRF_TWIM0->EVENTS_STOPPED = 0;
    NRF_TWIM0->EVENTS_SUSPENDED = 0;
    NRF_TWIM0->EVENTS_TXSTARTED = 0;
}
#if defined NRF52840
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler ()
{
#if defined HAL_SPIM_PERIPH_USED
#if HAL_SPIM_PERIPH_USED == 1
    hal_spim_Handler ();
#endif
#endif
#if defined HAL_TWIM_PERIPH_USED
#if HAL_TWIM_PERIPH_USED == 1
    hal_twim_Handler ();
#endif
#endif
//Clear events
    NRF_SPIM1->EVENTS_END = 0;
    NRF_SPIM1->EVENTS_ENDRX = 0;
    NRF_SPIM1->EVENTS_ENDTX = 0;
    NRF_SPIM1->EVENTS_STARTED = 0;
    NRF_SPIM1->EVENTS_STOPPED = 0;
    
    NRF_TWIM1->EVENTS_ERROR = 0;
    NRF_TWIM1->EVENTS_LASTRX = 0;
    NRF_TWIM1->EVENTS_LASTTX = 0;
    NRF_TWIM1->EVENTS_RXSTARTED = 0;
    NRF_TWIM1->EVENTS_STOPPED = 0;
    NRF_TWIM1->EVENTS_SUSPENDED = 0;
    NRF_TWIM1->EVENTS_TXSTARTED = 0;
}

void NFCT_IRQHandler (void)
{
}
#endif
void GPIOTE_IRQHandler (void)
{
#if defined GPIOTE_CH_USED_BUTTON_UI_PORT
    button_ui_gpiote_Handler ();
#endif
//Clear events
    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->EVENTS_IN[0] = 0;
    NRF_GPIOTE->EVENTS_IN[1] = 0;
    NRF_GPIOTE->EVENTS_IN[2] = 0;
    NRF_GPIOTE->EVENTS_IN[3] = 0;
    NRF_GPIOTE->EVENTS_IN[4] = 0;
    NRF_GPIOTE->EVENTS_IN[5] = 0;
    NRF_GPIOTE->EVENTS_IN[6] = 0;
    NRF_GPIOTE->EVENTS_IN[7] = 0;
}

void SAADC_IRQHandler (void)
{
//Clear events
}

void TIMER0_IRQHandler (void)
{
#if defined TIMER_USED_TSSP_IR_TX_1
#if TIMER_USED_TSSP_IR_TX_1 == 0
    tssp_ir_tx_timer1_Handler ();
#endif
#endif

#if defined TIMER_USED_TSSP_IR_TX_2
#if TIMER_USED_TSSP_IR_TX_2 == 0
    tssp_ir_tx_timer2_Handler ();
#endif
#endif

#if defined TIMER_USED_US_TIMER
#if TIMER_USED_US_TIMER == 0
    us_timer_timer_Handler ();
#endif
#endif

#if defined TIMER_USED_PROFILE_TIMER
#if TIMER_USED_PROFILE_TIMER == 0
    
#endif
#endif

#if defined TIMER_USED_SIMPLE_PWM
#if TIMER_USED_SIMPLE_PWM == 0
    
#endif
#endif
//Clear events
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
    NRF_TIMER0->EVENTS_COMPARE[2] = 0;
    NRF_TIMER0->EVENTS_COMPARE[3] = 0;
    NRF_TIMER0->EVENTS_COMPARE[4] = 0;
    NRF_TIMER0->EVENTS_COMPARE[5] = 0;
}

void TIMER1_IRQHandler (void)
{
#if defined TIMER_USED_TSSP_IR_TX_1
#if TIMER_USED_TSSP_IR_TX_1 == 1
    tssp_ir_tx_timer1_Handler ();
#endif
#endif

#if defined TIMER_USED_TSSP_IR_TX_2
#if TIMER_USED_TSSP_IR_TX_2 == 1
    tssp_ir_tx_timer2_Handler ();
#endif
#endif

#if defined TIMER_USED_US_TIMER
#if TIMER_USED_US_TIMER == 1
    us_timer_timer_Handler ();
#endif
#endif

#if defined TIMER_USED_PROFILE_TIMER
#if TIMER_USED_PROFILE_TIMER == 1
#endif
#endif

#if defined TIMER_USED_SIMPLE_PWM
#if TIMER_USED_SIMPLE_PWM == 1
#endif
#endif
//Clear events
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;
    NRF_TIMER1->EVENTS_COMPARE[2] = 0;
    NRF_TIMER1->EVENTS_COMPARE[3] = 0;
    NRF_TIMER1->EVENTS_COMPARE[4] = 0;
    NRF_TIMER1->EVENTS_COMPARE[5] = 0;
}

void TIMER2_IRQHandler (void)
{
#if defined TIMER_USED_TSSP_IR_TX_1
#if TIMER_USED_TSSP_IR_TX_1 == 2
    tssp_ir_tx_timer1_Handler ();
#endif
#endif

#if defined TIMER_USED_TSSP_IR_TX_2
#if TIMER_USED_TSSP_IR_TX_2 == 2
    tssp_ir_tx_timer2_Handler ();
#endif
#endif

#if defined TIMER_USED_US_TIMER
#if TIMER_USED_US_TIMER == 2
    us_timer_timer_Handler ();
#endif
#endif

#if defined TIMER_USED_PROFILE_TIMER
#if TIMER_USED_PROFILE_TIMER == 2
#endif
#endif

#if defined TIMER_USED_SIMPLE_PWM 
#if TIMER_USED_SIMPLE_PWM == 2
#endif
#endif
//Clear events
    NRF_TIMER2->EVENTS_COMPARE[0] = 0;
    NRF_TIMER2->EVENTS_COMPARE[1] = 0;
    NRF_TIMER2->EVENTS_COMPARE[2] = 0;
    NRF_TIMER2->EVENTS_COMPARE[3] = 0;
    NRF_TIMER2->EVENTS_COMPARE[4] = 0;
    NRF_TIMER2->EVENTS_COMPARE[5] = 0;
}

void RTC0_IRQHandler (void)
{
#if defined RTC_USED_PIR_SENSE
#if RTC_USED_PIR_SENSE == 0
#endif
#endif

#if defined RTC_USED_TSSP_DETECT
#if RTC_USED_TSSP_DETECT == 0
    tssp_detect_rtc_Handler ();
#endif
#endif

#if defined RTC_USED_MS_TIMER
#if RTC_USED_MS_TIMER == 0
    ms_timer_rtc_Handler ();
#endif
#endif
//Clear events
    NRF_RTC0->EVENTS_TICK = 0;
    NRF_RTC0->EVENTS_OVRFLW = 0;
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    NRF_RTC0->EVENTS_COMPARE[1] = 0;
    NRF_RTC0->EVENTS_COMPARE[2] = 0;
    NRF_RTC0->EVENTS_COMPARE[3] = 0;
}

void TEMP_IRQHandler (void)
{
}

void RNG_IRQHandler (void)
{
}

void ECB_IRQHandler (void)
{
}

void CCM_AAR_IRQHandler (void)
{
}

void WDT_IRQHandler (void)
{
    hal_wdt_Handler ();
    NRF_WDT->EVENTS_TIMEOUT = 0;
    (void) NRF_WDT->EVENTS_TIMEOUT;
}

void RTC1_IRQHandler (void)
{
#if defined RTC_USED_PIR_SENSE
#if RTC_USED_PIR_SENSE == 1
#endif
#endif

#if defined RTC_USED_TSSP_DETECT
#if RTC_USED_TSSP_DETECT == 1
    tssp_detect_rtc_Handler ();
#endif
#endif

#if defined RTC_USED_MS_TIMER
#if RTC_USED_MS_TIMER == 1
    ms_timer_rtc_Handler ();
#endif
#endif
//Clear events
    NRF_RTC1->EVENTS_TICK = 0;
    NRF_RTC1->EVENTS_OVRFLW = 0;
    NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->EVENTS_COMPARE[1] = 0;
    NRF_RTC1->EVENTS_COMPARE[2] = 0;
    NRF_RTC1->EVENTS_COMPARE[3] = 0;
}

void QDEC_IRQHandler (void)
{
}

void COMP_LPCOMP_IRQHandler (void)
{
}

void SWI0_EGU0_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 0
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 0
    sensebe_ble_swi_Handler ();
#endif
#endif

#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 0
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
    for(uint32_t x; x < 16; x++)
    {
        NRF_EGU0->EVENTS_TRIGGERED[x] = 0;
    }

}

void SWI1_EGU1_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 1
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 1
    sensebe_ble_swi_Handler ();
#endif
#endif

#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 1
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
    for(uint32_t x; x < 16; x++)
    {
        NRF_EGU1->EVENTS_TRIGGERED[x] = 0;
    }
}
void SWI2_EGU2_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 2
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 2
    sensebe_ble_swi_Handler ();
#endif
#endif

#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 2
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
#if defined NRF52840
    for(uint32_t x; x < 16; x++)
    {
        NRF_EGU2->EVENTS_TRIGGERED[x] = 0;
    }
#endif
}

void SWI3_EGU3_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 3
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 3
    sensebe_ble_swi_Handler ();
#endif
#endif

#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 3
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
#if defined NRF52840
    for(uint32_t x; x < 16; x++)
    {
        NRF_EGU3->EVENTS_TRIGGERED[x] = 0;
    }
#endif
}

void SWI4_EGU4_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 4
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 4
    sensebe_ble_swi_Handler ();
#endif
#endif


#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 4
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
#if defined NRF52840
    {
    for(uint32_t x; x < 16; x++)
        NRF_EGU4->EVENTS_TRIGGERED[x] = 0;
    }
#endif
}

void SWI5_EGU5_IRQHandler (void)
{
#if defined EVT_SD_HANDLER_SWI_USED
#if EVT_SD_HANDLER_SWI_USED == 5
    evt_sd_handler_swi_Handler ();
#endif
#endif

#if defined SWI_SENSEBE_BLE_USED
#if SWI_SENSEBE_BLE_USED == 5
    sensebe_ble_swi_Handler ();
#endif
#endif


#if defined EGU_USED_TSSP_DETECT
#if EGU_USED_TSSP_DETECT == 5
    tssp_detect_swi_Handler ();
#endif
#endif
//Clear events
#if defined NRF52840
    for(uint32_t x; x < 16; x++)
    {
        NRF_EGU5->EVENTS_TRIGGERED[x] = 0;
    }
#endif
}

#if defined NRF52840
void TIMER3_IRQHandler (void)
{
#if defined TIMER_USED_TSSP_IR_TX_1
#if TIMER_USED_TSSP_IR_TX_1 == 3
    tssp_ir_tx_timer1_Handler ();
#endif
#endif

#if defined TIMER_USED_TSSP_IR_TX_2
#if TIMER_USED_TSSP_IR_TX_2 == 3
    tssp_ir_tx_timer2_Handler ();
#endif
#endif

#if defined TIMER_USED_US_TIMER
#if TIMER_USED_US_TIMER == 3
    us_timer_timer_Handler ();
#endif
#endif

#if defined TIMER_USED_PROFILE_TIMER
#if TIMER_USED_PROFILE_TIMER == 3
#endif
#endif

#if defined TIMER_USED_SIMPLE_PWM
#if TIMER_USED_SIMPLE_PWM == 3
#endif
#endif
//Clear events
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->EVENTS_COMPARE[1] = 0;
    NRF_TIMER3->EVENTS_COMPARE[2] = 0;
    NRF_TIMER3->EVENTS_COMPARE[3] = 0;
    NRF_TIMER3->EVENTS_COMPARE[4] = 0;
    NRF_TIMER3->EVENTS_COMPARE[5] = 0;
}

void TIMER4_IRQHandler (void)
{
#if defined TIMER_USED_TSSP_IR_TX_1
#if TIMER_USED_TSSP_IR_TX_1 == 4
    tssp_ir_tx_timer1_Handler ();
#endif
#endif

#if defined TIMER_USED_TSSP_IR_TX_2
#if TIMER_USED_TSSP_IR_TX_2 == 4
    tssp_ir_tx_timer2_Handler ();
#endif
#endif

#if defined TIMER_USED_US_TIMER
#if TIMER_USED_US_TIMER == 4
    us_timer_timer_Handler ();
#endif
#endif

#if defined TIMER_USED_PROFILE_TIMER
#if TIMER_USED_PROFILE_TIMER == 4
#endif
#endif

#if defined TIMER_USED_SIMPLE_PWM
#if TIMER_USED_SIMPLE_PWM == 4
#endif
#endif
//Clear events
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    NRF_TIMER4->EVENTS_COMPARE[1] = 0;
    NRF_TIMER4->EVENTS_COMPARE[2] = 0;
    NRF_TIMER4->EVENTS_COMPARE[3] = 0;
    NRF_TIMER4->EVENTS_COMPARE[4] = 0;
    NRF_TIMER4->EVENTS_COMPARE[5] = 0;
}
#endif

void PWM0_IRQHandler (void)
{
#if defined HAL_PWM_PERIPH_USED 
#if HAL_PWM_PERIPH_USED == 0
    hal_pwm_Handler ();
#endif
#endif
//Clear events
    NRF_PWM0->EVENTS_LOOPSDONE = 0;
    NRF_PWM0->EVENTS_PWMPERIODEND = 0;
    NRF_PWM0->EVENTS_SEQEND[0] = 0;
    NRF_PWM0->EVENTS_SEQEND[1] = 0;
    NRF_PWM0->EVENTS_SEQEND[2] = 0;
    NRF_PWM0->EVENTS_SEQEND[3] = 0;
    NRF_PWM0->EVENTS_SEQSTARTED[0] = 0;
    NRF_PWM0->EVENTS_SEQSTARTED[1] = 0;
    NRF_PWM0->EVENTS_SEQSTARTED[2] = 0;
    NRF_PWM0->EVENTS_SEQSTARTED[3] = 0;
    NRF_PWM0->EVENTS_STOPPED = 0;
}

void PDM_IRQHandler (void)
{
}

void MWU_IRQHandler (void)
{
}

#if defined NRF52840
void PWM1_IRQHandler (void)
{
#if defined HAL_PWM_PERIPH_USED 
#if HAL_PWM_PERIPH_USED == 1
    hal_pwm_Handler ();
#endif
#endif
    NRF_PWM1->EVENTS_LOOPSDONE = 0;
    NRF_PWM1->EVENTS_PWMPERIODEND = 0;
    NRF_PWM1->EVENTS_SEQEND = 0;
    NRF_PWM1->EVENTS_SEQSTARTED = 0;
    NRF_PWM1->EVENTS_STOPPED = 0;
}

void PWM2_IRQHandler (void)
{
#if defined HAL_PWM_PERIPH_USED 
#if HAL_PWM_PERIPH_USED == 2
#endif
#endif
//Clear events
    NRF_PWM2->EVENTS_LOOPSDONE = 0;
    NRF_PWM2->EVENTS_PWMPERIODEND = 0;
    NRF_PWM2->EVENTS_SEQEND = 0;
    NRF_PWM2->EVENTS_SEQSTARTED = 0;
    NRF_PWM2->EVENTS_STOPPED = 0;
}

void SPIM2_SPIS2_SPI2_IRQHandler (void)
{
#if defined HAL_SPIM_PERIPH_USED
#if HAL_SPIM_PERIPH_USED == 2
#endif
#endif
    
#if defined HAL_TWIM_PERIPH_USED 
#if HAL_TWIM_PERIPH_USED == 2
#endif
#endif
//Clear events
    NRF_SPIM2->EVENTS_END = 0;
    NRF_SPIM2->EVENTS_ENDRX = 0;
    NRF_SPIM2->EVENTS_ENDTX = 0;
    NRF_SPIM2->EVENTS_STARTED = 0;
    NRF_SPIM2->EVENTS_STOPPED = 0;
    
    NRF_TWIM2->EVENTS_ERROR = 0;
    NRF_TWIM2->EVENTS_LASTRX = 0;
    NRF_TWIM2->EVENTS_LASTTX = 0;
    NRF_TWIM2->EVENTS_RXSTARTED = 0;
    NRF_TWIM2->EVENTS_STOPPED = 0;
    NRF_TWIM2->EVENTS_SUSPENDED = 0;
    NRF_TWIM2->EVENTS_TXSTARTED = 0;
}

void RTC2_IRQHandler (void)
{
#if defined RTC_USED_PIR_SENSE
#if RTC_USED_PIR_SENSE == 2
#endif
#endif

#if defined RTC_USED_TSSP_DETECT
#if RTC_USED_TSSP_DETECT == 2
#endif
#endif

#if defined RTC_USED_MS_TIMER 
#if RTC_USED_MS_TIMER == 2
#endif
#endif
//Clear events
    NRF_RTC2->EVENTS_TICK = 0;
    NRF_RTC2->EVENTS_OVRFLW = 0;
    NRF_RTC2->EVENTS_COMPARE = 0;
}

void I2S_IRQHandler (void)
{
}

void FPU_IRQHandler (void)
{
}
#endif



