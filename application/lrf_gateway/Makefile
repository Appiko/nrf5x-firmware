
BOARD           := BOARD_SENSEELE_SMA_REV1
SLNO            := 682073614
BLE_REQD        := BLE_STACK_SUPPORT_REQD
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
DEBUGGER        := BMP
LOGGER          := LOG_UART_PRINTF
FW_VER_VAL      := 0
MS_TIMER_FREQ   := 32768
TSSP_DETECT_FREQ := 1000
SD_USED         := 
SD_VER          := 6.0.0
CONFIG_HEADER	:= 0
RADIO_XTAL_FREQ := 32000000

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase
RADIO_DIR	= $(CODEBASE_DIR)/rf_lib

INCLUDEDIRS	= ./
INCLUDEDIRS += ../rf_data_tx_rx/S2LP_Library/Inc
INCLUDEDIRS += ../rf_data_tx_rx/
INCLUDEDIRS += $(RADIO_DIR)/
INCLUDEDIRS += $(RADIO_DIR)/ti_radio_lib

INCLUDEDIRS	+= $(PLATFORM_DIR)
ifneq ($(SD_USED_LC),blank)
INCLUDEDIRS += $(SDK_DIR)/softdevice/$(SD_USED)/headers
endif
INCLUDEDIRS += $(CODEBASE_DIR)/hal
INCLUDEDIRS += $(CODEBASE_DIR)/segger_rtt
INCLUDEDIRS += $(CODEBASE_DIR)/sd_assist
INCLUDEDIRS += $(CODEBASE_DIR)/peripheral_modules
INCLUDEDIRS += $(CODEBASE_DIR)/util

#INCLUDEDIRS += $(CODEBASE_DIR)/ti_radio_lib
#INCLUDEDIRS += $(CODEBASE_DIR)/ti_radio_lib/cc112x_drv

C_SRC_DIRS = .

C_SRC_DIRS += ../rf_data_tx_rx/
C_SRC_DIRS += ../rf_data_tx_rx/S2LP_Library/Src

C_SRC_DIRS += $(CODEBASE_DIR)/segger_rtt
C_SRC_DIRS += $(CODEBASE_DIR)/hal
C_SRC_DIRS += $(CODEBASE_DIR)/sd_assist
C_SRC_DIRS += $(CODEBASE_DIR)/peripheral_modules
C_SRC_DIRS += $(CODEBASE_DIR)/util
C_SRC_DIRS += $(RADIO_DIR)/
C_SRC_DIRS += $(RADIO_DIR)/ti_radio_lib
#
#C_SRC_DIRS += $(CODEBASE_DIR)/ti_radio_lib
#C_SRC_DIRS += $(CODEBASE_DIR)/ti_radio_lib/cc112x_drv


C_SRC = main.c
C_SRC += nrf_assert.c
#C_SRC += app_error.c
C_SRC += hal_clocks.c ms_timer.c
ifeq ($(LOGGER), LOG_SEGGER_RTT)
C_SRC += SEGGER_RTT.c SEGGER_RTT_printf.c
else ifeq ($(LOGGER), LOG_UART_DMA_PRINTF)
C_SRC += uart_printf.c tinyprintf.c
else ifeq ($(LOGGER), LOG_UART_PRINTF)
C_SRC += hal_uart.c tinyprintf.c
else ifeq ($(LOGGER), LOG_TEENSY)
C_SRC += hal_uart.c tinyprintf.c
endif
C_SRC += hal_wdt.c
C_SRC += nrf_util.c 

C_SRC += hal_spim.c

C_SRC += byte_frame.c
C_SRC += simple_adc.c

#C_SRC += SDK_EVAL_Spi_Driver.c
#
#C_SRC += S2LP_Commands.c
#C_SRC += S2LP_Csma.c
#C_SRC += S2LP_Fifo.c
#C_SRC += S2LP_General.c
#C_SRC += S2LP_Gpio.c
#C_SRC += S2LP_PacketHandler.c
#C_SRC += S2LP_PktBasic.c
#C_SRC += S2LP_PktStack.c
#C_SRC += S2LP_PktWMbus.c
#C_SRC += S2LP_Qi.c
#C_SRC += S2LP_Radio.c
#C_SRC += S2LP_Types.c
#C_SRC += S2LP_Timer.c
#C_SRC += S2LP_Timer_ex.c

#C_SRC += hal_spi_rf.c
#C_SRC += cc112x_drv.c
#C_SRC += cc112x_utils.c
C_SRC += rf_comm.c
C_SRC += rf_spi_hw.c
C_SRC += spi_rf_nrf52.c

#Gets the name of the application folder
APPLN = $(shell basename $(PWD))

CFLAGS_APP = -DDEBUG
CFLAGS_APP += -D$(BLE_REQD)
CFLAGS_APP += -D$(PINRESET_REQD)
CFLAGS_APP += -D$(LOGGER)
CFLAGS_APP += -DFW_VER=$(FW_VER_VAL)
CFLAGS_APP += -DMS_TIMER_FREQ=$(MS_TIMER_FREQ)
CFLAGS_APP += -DTSSP_DETECT_FREQ=$(TSSP_DETECT_FREQ)
CFLAGS_APP += -DSYS_CFG_PRESENT=$(CONFIG_HEADER)


#Lower case of BOARD
BOARD_HEADER  = $(shell echo $(BOARD) | tr A-Z a-z)
include $(PLATFORM_DIR)/Makefile.$(BOARD_HEADER)

include ../Makefile.common
