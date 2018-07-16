
BOARD           := BOARD_SENSEPI_REV4
SLNO            := 681592281
NO_NFC_REQD     := CONFIG_NFCT_PINS_AS_GPIOS
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
DEBUGGER		:= BMP
LOGGER			:= LOG_UART_PRINTF

SD_USED         := blank

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase

INCLUDEDIRS	= .
INCLUDEDIRS	+= $(PLATFORM_DIR)
INCLUDEDIRS += $(CODEBASE_DIR)/hal/
INCLUDEDIRS += $(SDK_DIR)
INCLUDEDIRS += $(CODEBASE_DIR)/nrf_core/
INCLUDEDIRS += $(SDK_DIR)/toolchain/cmsis/include/
INCLUDEDIRS += $(CODEBASE_DIR)/peripheral_modules/
INCLUDEDIRS += $(CODEBASE_DIR)/util/

C_SRC_DIRS	= .
C_SRC_DIRS	+= $(CODEBASE_DIR)/peripheral_modules/
C_SRC_DIRS	+= $(CODEBASE_DIR)/util/
C_SRC_DIRS	+= $(CODEBASE_DIR)/hal/

C_SRC  = main.c
C_SRC += mcp4012_x.c
C_SRC += profiler_timer.c
C_SRC += ms_timer.c
C_SRC += hal_clocks.c
C_SRC += simple_adc.c
C_SRC += nrf_assert.c
C_SRC += uart_printf.c
C_SRC += tinyprintf.c
C_SRC += hw_testing_app.c
C_SRC += product_id.c

#Gets the name of the application folder
APPLN = $(shell basename $(PWD))

CFLAGS_APP = -DDEBUG
CFLAGS_APP += -D$(NO_NFC_REQD)
CFLAGS_APP += -D$(PINRESET_REQD)
CFLAGS_APP += -D$(LOGGER)

#Lower case of BOARD
BOARD_HEADER  = $(shell echo $(BOARD) | tr A-Z a-z)
include $(PLATFORM_DIR)/Makefile.$(BOARD_HEADER)

include ../Makefile.common
