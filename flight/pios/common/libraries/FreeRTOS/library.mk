#
# Rules to add FreeRTOS to a PiOS target
#
# Note that the PIOS target-specific makefile will detect that FREERTOS_DIR
# has been defined and add in the target-specific pieces separately.
#

UNAME := $(shell uname)
# Here and everywhere if not Linux or Mac then assume Windows
ifeq ($(filter Linux Darwin, $(UNAME)), )
    UNAME := Windows
endif 

ifeq ($(UNAME), Windows)
FREERTOS_DIR	:= $(dir $(lastword $(MAKEFILE_LIST)))Source
else
FREERTOS_DIR	:= $(dir $(lastword $(MAKEFILE_LIST)))/Source
endif
SRC		+= $(wildcard $(FREERTOS_DIR)/*.c)
EXTRAINCDIRS	+= $(FREERTOS_DIR)/include
