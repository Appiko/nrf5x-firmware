
BOARD           := BOARD_HACKABLE
SLNO            := 682073614
BLE_REQD        := BLE_STACK_SUPPORT_REQD
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
DEBUGGER        := BMP
LOGGER			:= LOG_UART_PRINTF

SD_USED         := s132

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase

INCLUDEDIRS	= .
INCLUDEDIRS	+= $(PLATFORM_DIR)
ifneq ($(SD_USED_LC),blank)
INCLUDEDIRS += $(SDK_DIR)/softdevice/$(SD_USED)/headers
endif
INCLUDEDIRS += $(CODEBASE_DIR)/hal
INCLUDEDIRS += $(CODEBASE_DIR)/segger_rtt
INCLUDEDIRS += $(CODEBASE_DIR)/sd_assist
INCLUDEDIRS += $(CODEBASE_DIR)/peripheral_modules
INCLUDEDIRS += $(CODEBASE_DIR)/util

C_SRC_DIRS = .
C_SRC_DIRS += $(CODEBASE_DIR)/segger_rtt
C_SRC_DIRS += $(CODEBASE_DIR)/hal
C_SRC_DIRS += $(CODEBASE_DIR)/sd_assist
C_SRC_DIRS += $(CODEBASE_DIR)/peripheral_modules
C_SRC_DIRS += $(CODEBASE_DIR)/util

C_SRC = main.c
C_SRC += nrf_assert.c app_error.c
C_SRC += hal_clocks.c ms_timer.c
ifeq ($(LOGGER), LOG_SEGGER_RTT)
C_SRC += SEGGER_RTT.c SEGGER_RTT_printf.c
else ifeq ($(LOGGER), LOG_UART_DMA_PRINTF)
C_SRC += uart_printf.c tinyprintf.c hal_uart.c
else ifeq ($(LOGGER), LOG_UART_PRINTF)
C_SRC += hal_uart.c tinyprintf.c
else
endif
C_SRC += profiler_timer.c
C_SRC += nrf_util.c
C_SRC += sd_evt_handler.c

#Gets the name of the application folder
APPLN = $(shell basename $(PWD))

CFLAGS_APP = -DDEBUG
CFLAGS_APP += -D$(BLE_REQD)
CFLAGS_APP += -D$(PINRESET_REQD)
CFLAGS_APP += -D$(LOGGER)

#Lower case of BOARD
BOARD_HEADER  = $(shell echo $(BOARD) | tr A-Z a-z)
include $(PLATFORM_DIR)/Makefile.$(BOARD_HEADER)

include ../Makefile.common
