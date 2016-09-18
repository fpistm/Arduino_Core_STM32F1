#
#  Copyright (c) 2011 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

# Makefile for compiling libchip
.SUFFIXES: .o .a .c .s
SUB_MAKEFILES= debug.mk gcc.mk release.mk stm32f1.mk

LIBNAME=libstm32f1
TOOLCHAIN=gcc

ifeq ($(OS),Windows_NT)
DEV_NUL=NUL
else
DEV_NUL=/dev/null
endif

ifeq ($(CHIP),)
$(error CHIP not defined)
endif

#CHIP_NAME=$(subst __,,$(CHIP))
#CHIP_NAME=$(subst __,,$(call lc,$(CHIP)))

#-------------------------------------------------------------------------------
# Path
#-------------------------------------------------------------------------------

# Board options
ifeq ($(CHIP), __NUCLEO_F103RB__)
CHIP_NAME=nucleo-f103rb
CHIP_SERIE=STM32F1xx
CFLAGS += -DSTM32F103xB
# Output directories
OUTPUT_BIN = ../../../variants/STM32F103RB-Nucleo
#Startup file
CHIP_STARTUP_FILE=startup_stm32f103xb.s
else ifeq ($(CHIP), __BLUEPILL_F103C8__)
CHIP_NAME=bluepill-f103c8
CHIP_SERIE=STM32F1xx
CFLAGS += -DSTM32F103xB
# Output directories
OUTPUT_BIN = ../../../variants/STM32F103C8-BluePill
#Startup file
CHIP_STARTUP_FILE=startup_stm32f103xb.s
else
$(error CHIP not recognized)
endif

# Libraries
PROJECT_BASE_PATH = ..
CMSIS_ROOT_PATH = $(PROJECT_BASE_PATH)/../Drivers/CMSIS
HAL_ROOT_PATH = $(PROJECT_BASE_PATH)/../Drivers/STM32F1xx_HAL_Driver

CMSIS_ARM_PATH=$(CMSIS_ROOT_PATH)/Include
CMSIS_ST_PATH=$(CMSIS_ROOT_PATH)/Device/ST/
CMSIS_CHIP_PATH=$(CMSIS_ROOT_PATH)/Device/ST/$(CHIP_SERIE)
STARTUP_FILE_PATH=$(CMSIS_CHIP_PATH)/Source/Templates/gcc

#-------------------------------------------------------------------------------
# Files
#-------------------------------------------------------------------------------
vpath %.h $(PROJECT_BASE_PATH)/include $(CMSIS_ST_PATH) $(CMSIS_CHIP_PATH)/include
vpath %.c $(PROJECT_BASE_PATH)/source $(CMSIS_ARM_PATH) $(CMSIS_CHIP_PATH)/source

VPATH+=$(HAL_ROOT_PATH)/Src
VPATH+=$(PROJECT_BASE_PATH)/source
VPATH+=$(CMSIS_ARM_PATH)
VPATH+=$(CMSIS_CHIP_PATH)/Include
VPATH+=$(CMSIS_CHIP_PATH)/Source/
VPATH+=$(STARTUP_FILE_PATH)

INCLUDES = -I$(PROJECT_BASE_PATH)
INCLUDES += -I$(HAL_ROOT_PATH)/Inc
INCLUDES += -I$(PROJECT_BASE_PATH)/include
INCLUDES += -I$(CMSIS_ARM_PATH)
INCLUDES += -I$(CMSIS_ST_PATH)
INCLUDES += -I$(CMSIS_CHIP_PATH)/Include
INCLUDES += -I$(STARTUP_FILE_PATH)

#-------------------------------------------------------------------------------
ifdef DEBUG
include debug.mk
else
include release.mk
endif

#-------------------------------------------------------------------------------
# Tools
#-------------------------------------------------------------------------------

include $(TOOLCHAIN).mk

#-------------------------------------------------------------------------------
ifdef DEBUG
OUTPUT_OBJ=debug
OUTPUT_LIB=$(LIBNAME)_$(CHIP_NAME)_$(TOOLCHAIN)_dbg.a
else
OUTPUT_OBJ=release
OUTPUT_LIB=$(LIBNAME)_$(CHIP_NAME)_$(TOOLCHAIN)_rel.a
endif

OUTPUT_PATH=$(OUTPUT_OBJ)_$(CHIP_NAME)

#-------------------------------------------------------------------------------
# C source files and objects
#-------------------------------------------------------------------------------
C_SRC=$(wildcard $(HAL_ROOT_PATH)/Src/*.c)
C_SRC+=$(wildcard $(PROJECT_BASE_PATH)/source/*.c)
C_SRC+=$(wildcard $(CMSIS_CHIP_PATH)/source/*.c)
C_SRC+=$(wildcard $(CMSIS_CHIP_PATH)/source/gcc/*.c)

C_OBJ_TEMP=$(patsubst %.c, %.o, $(notdir $(C_SRC)))

# during development, remove some files
C_OBJ_FILTER=

C_OBJ=$(filter-out $(C_OBJ_FILTER), $(C_OBJ_TEMP))

#-------------------------------------------------------------------------------
# Assembler source files and objects
#-------------------------------------------------------------------------------
A_SRC=$(wildcard $(PROJECT_BASE_PATH)/source/*.s)
A_SRC+=$(wildcard $(STARTUP_FILE_PATH)/$(CHIP_STARTUP_FILE))


A_OBJ_TEMP=$(patsubst %.s, %.o, $(notdir $(A_SRC)))

# during development, remove some files
A_OBJ_FILTER=

A_OBJ=$(filter-out $(A_OBJ_FILTER), $(A_OBJ_TEMP))

#-------------------------------------------------------------------------------
# Rules
#-------------------------------------------------------------------------------
all: $(CHIP)

$(CHIP): create_output $(OUTPUT_LIB)

.PHONY: create_output
create_output:
	@echo ------------------------------------------------------------------------
	@echo --- Preparing $(CHIP) files $(OUTPUT_PATH) to $(OUTPUT_BIN)
#	@echo -------------------------
#	@echo *$(C_SRC)
#	@echo -------------------------
#	@echo *$(C_OBJ)
#	@echo -------------------------
#	@echo *$(addprefix $(OUTPUT_PATH)/, $(C_OBJ))
#	@echo -------------------------
#	@echo *$(A_SRC)
#	@echo -------------------------

	-@mkdir $(subst /,$(SEP),$(OUTPUT_BIN)) 1>$(DEV_NUL) 2>&1
	-@mkdir $(OUTPUT_PATH) 1>$(DEV_NUL) 2>&1
	@echo ------------------------------------------------------------------------

$(addprefix $(OUTPUT_PATH)/,$(C_OBJ)): $(OUTPUT_PATH)/%.o: %.c
#	"$(CC)" -v -c $(CFLAGS) -Wa,aln=$(subst .o,.s,$@) $< -o $@
	@echo "  CC       " $< $(CFLAGS)
	@"$(CC)" -c $(CFLAGS) $< -o $@
#	"$(CC)" -c $(CFLAGS) $< -o $@

$(addprefix $(OUTPUT_PATH)/,$(A_OBJ)): $(OUTPUT_PATH)/%.o: %.s
	@echo "  AS       " $< $(ASFLAGS)
	@"$(AS)" -c $(ASFLAGS) $< -o $@
#	@"$(AS)" -c $(ASFLAGS) $< -o $@

$(OUTPUT_LIB): $(addprefix $(OUTPUT_PATH)/, $(C_OBJ)) $(addprefix $(OUTPUT_PATH)/, $(A_OBJ))
	@"$(AR)" -r "$(OUTPUT_BIN)/$@" $^
	@"$(NM)" "$(OUTPUT_BIN)/$@" > "$(OUTPUT_BIN)/$@.txt"

.PHONY: clean
clean:
	@echo ------------------------------------------------------------------------
	@echo --- Cleaning $(CHIP) files $(OUTPUT_PATH) $(subst /,$(SEP),$(OUTPUT_BIN)/$(OUTPUT_LIB))
	-@$(RM) $(OUTPUT_PATH) 1>$(DEV_NUL) 2>&1
	-@$(RM) $(subst /,$(SEP),$(OUTPUT_BIN)/$(OUTPUT_LIB)) 1>$(DEV_NUL) 2>&1
	-@$(RM) $(subst /,$(SEP),$(OUTPUT_BIN)/$(OUTPUT_LIB)).txt 1>$(DEV_NUL) 2>&1
	@echo ------------------------------------------------------------------------

# dependencies
$(addprefix $(OUTPUT_PATH)/,$(C_OBJ)): $(OUTPUT_PATH)/%.o: $(PROJECT_BASE_PATH)/chip.h $(wildcard $(PROJECT_BASE_PATH)/include/*.h) $(wildcard $(CMSIS_BASE_PATH)/*.h)
