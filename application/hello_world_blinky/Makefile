
BOARD				:= BOARD_PCA10040
SLNO				:= 682073614
BLE_REQD    := BLE_STACK_SUPPORT_REQD
NO_NFC_REQD := CONFIG_NFCT_PINS_AS_GPIOS
PINRESET_REQD := CONFIG_GPIO_AS_PINRESET

SDK_DIR         = ../../SDK_components
DOC_DIR         = ../../doc
PLATFORM_DIR    = ../../platform
CODEBASE_DIR    = ../../codebase

INCLUDEDIRS	= .
INCLUDEDIRS	+= $(PLATFORM_DIR)
INCLUDEDIRS += $(SDK_DIR)/drivers_nrf/delay
INCLUDEDIRS += $(SDK_DIR)/drivers_nrf/hal

C_SRC_DIRS	= .

C_SRC  = main.c

APPLN  = hello_world_blinky

CFLAGS_APP = -DDEBUG
CFLAGS_APP += -D$(BLE_REQD)

include ../Makefile.common
