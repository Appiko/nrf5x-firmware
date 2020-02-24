
BOARD           := BOARD_SENSEBLE_REV1
SLNO            := 681592281
NO_NFC_REQD     := CONFIG_NFCT_PINS_AS_GPIOS
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
DEBUGGER	:= JLINK
LOGGER          := LOG_SEGGER_RTT


SD_USED         := blank

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase

INCLUDEDIRS	= .
INCLUDEDIRS	+= $(PLATFORM_DIR)
INCLUDEDIRS	+= $(CODEBASE_DIR)/hal/
INCLUDEDIRS	+= $(CODEBASE_DIR)/util/
INCLUDEDIRS	+= $(CODEBASE_DIR)/peripheral_modules/
INCLUDEDIRS	+= $(CODEBASE_DIR)/segger_rtt/



C_SRC_DIRS	= .
C_SRC_DIRS	+= $(CODEBASE_DIR)/peripheral_modules/
C_SRC_DIRS	+= $(CODEBASE_DIR)/hal/
C_SRC_DIRS	+= $(CODEBASE_DIR)/util/
C_SRC_DIRS	+= $(CODEBASE_DIR)/segger_rtt/

C_SRC  = main.c
C_SRC += hal_uarte.c tinyprintf.c hal_clocks.c
C_SRC += nrf_util.c ms_timer.c SEGGER_RTT.c SEGGER_RTT_printf.c
C_SRC += minmea.c

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
