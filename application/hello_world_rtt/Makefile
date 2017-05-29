
BOARD           := BOARD_PCA10040
SLNO            := 682073614
BLE_REQD        := BLE_STACK_SUPPORT_REQD
NO_NFC_REQD     := CONFIG_NFCT_PINS_AS_GPIOS
PINRESET_REQD   := CONFIG_GPIO_AS_PINRESET
#This application will only work with the JLink Segger debugger
DEBUGGER		:= JLINK

ifneq ($(DEBUGGER), JLINK)
${error "This application will only work with the JLink debugger as RTT is used for printing messages."}
endif 

SD_USED         := blank

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase

INCLUDEDIRS	= .
INCLUDEDIRS	+= $(PLATFORM_DIR)
INCLUDEDIRS += $(CODEBASE_DIR)/segger_rtt
INCLUDEDIRS += $(CODEBASE_DIR)/hal

C_SRC_DIRS = .
C_SRC_DIRS += $(CODEBASE_DIR)/segger_rtt

C_SRC = main.c
C_SRC += SEGGER_RTT.c SEGGER_RTT_printf.c #SEGGER_RTT_Syscalls_GCC.c

#Gets the name of the application folder
APPLN = $(shell basename $(PWD))

CFLAGS_APP = -DDEBUG
CFLAGS_APP += -D$(BLE_REQD)
CFLAGS_APP += -D$(NO_NFC_REQD)
CFLAGS_APP += -D$(PINRESET_REQD)

#Lower case of BOARD
BOARD_HEADER  = $(shell echo $(BOARD) | tr A-Z a-z)
include $(PLATFORM_DIR)/Makefile.$(BOARD_HEADER)

include ../Makefile.common
