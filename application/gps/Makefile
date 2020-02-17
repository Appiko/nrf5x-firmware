
BOARD           := BOARD_SENSEBLE_REV1
SLNO            := 682073614
NO_NFC_REQD     := CONFIG_NFCT_PINS_AS_GPIOS
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
DEBUGGER	:= BMP
LOGGER          := LOG_UART_PRINTF


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




C_SRC_DIRS	= .
C_SRC_DIRS	+= $(CODEBASE_DIR)/peripheral_modules/
C_SRC_DIRS	+= $(CODEBASE_DIR)/hal/
C_SRC_DIRS	+= $(CODEBASE_DIR)/util/

C_SRC  = main.c
C_SRC += hal_uart.c tinyprintf.c
C_SRC += nrf_util.c
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
