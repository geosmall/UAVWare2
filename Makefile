##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.14.1] date: [Sat Jul 30 06:49:33 EDT 2022]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
# TARGET ?= revolution
TARGET ?= nucleo_f411re


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
SRC_DIR = flight
TARGET_DIR = flight/targets/$(TARGET)
BUILD_DIR = build

######################################
# source
######################################

# Generate from source file list
INPUT_FILE = file_list.txt
# TARGET_INPUT_FILE += $(TARGET_DIR)/file_list.txt
# $(info $$INPUT_FILE := [${INPUT_FILE}])

SRCS := $(addprefix $(SRC_DIR)/, $(file < $(INPUT_FILE)))
# SRCS += $(addprefix $(TARGET_DIR)/, $(file < $(TARGET_INPUT_FILE)))
# $(info $$SRCS := [${SRCS}])

C_SOURCES := $(filter %.c,$(SRCS))
# $(info $$C_SOURCES := [${C_SOURCES}])
CPP_SOURCES := $(filter %.cpp,$(SRCS))
# $(info $$CPP_SOURCES := [${CPP_SOURCES}])
ASM_SOURCES := $(filter %.s,$(SRCS))
# $(info $$ASM_SOURCES := [${ASM_SOURCES}])


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS = \
-DUSE_FULL_LL_DRIVER \
-DLFS_CONFIG=lfs_config.h \
-DLFS_NO_MALLOC

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-I./flight/uvos \
-I./flight/uvos/inc \
-I./flight/uvos/common/libraries/dosfs \
-I./flight/uvos/common/libraries/littleFS \
-I./flight/uvos/common/libraries/FreeRTOS/Source/Include \
-I./flight/uvos/common/libraries/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I./flight/uvos/stm32f4xx/libraries/CMSIS/Include \
-I./flight/uvos/stm32f4xx/libraries/CMSIS/Device/ST/STM32F4xx/Include \
-I./flight/uvos/stm32f4xx/libraries/STM32F4xx_HAL_Driver/Inc \
-I./flight/uvos/stm32f4xx/libraries/STM32F4xx_StdPeriph_Driver/inc \
-I./flight/uavobjects/inc \
-I./flight/libraries/inc \
-I./flight/libraries/math \
-I./flight/libraries/mavlink \
-I./flight/libraries/printf \
-I./flight/targets/$(TARGET) \
-I./flight/modules/System/inc \
-I./flight/UAVWare \
-I./flight/objects \
-I./flight

# CFLAGS1 = -mapcs-frame -fomit-frame-pointer
# CFLAGS1 += -Wall -Wextra -Wfloat-equal -Wdouble-promotion -Wshadow -Werror
# CFLAGS2 = -fdata-sections -ffunction-sections
# CFLAGS2 += -std=gnu99 -Wunsuffixed-float-constants -DPIOS_ENABLE_CXX

# CPPFLAGS1 = -mapcs-frame -fomit-frame-pointer
# CPPFLAGS1 += -Wall -Wextra -Wfloat-equal -Wdouble-promotion -Wshadow -Werror
# CPPFLAGS2 = -fdata-sections -ffunction-sections
# CPPFLAGS2 += -DPIOS_ENABLE_CXX -fno-rtti -fno-exceptions -std=c++11 -fno-use-cxa-atexit

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -std=gnu99 -fdata-sections -ffunction-sections -Wno-address-of-packed-member -Wno-cast-align

CPPFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -std=c++11 -fdata-sections -ffunction-sections -Wno-address-of-packed-member -Wno-cast-align

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
CPPFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Include target specific files and options
include $(TARGET_DIR)/target.mk
# $(info $$C_DEFS := [${C_DEFS}])

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(TARGET_DIR)/ldscript.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))


$(BUILD_DIR)/%.o: %.c Makefile file_list.txt | $(BUILD_DIR)
	$(info CC $@)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile file_list.txt | $(BUILD_DIR)
	$(info CPP $@)
	$(CXX) -c $(CPPFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile file_list.txt | $(BUILD_DIR)
	$(info AS $@)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile file_list.txt
	$(info LD linking)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@ -Wl,--print-memory-usage
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

# Flash firmware into chip 
.PHONY: flash
flash:
	"C:/Program Files/SEGGER/Ozone/Ozone.exe" $(TARGET).jdebug


#######################################
# clean up
#######################################
clean:
ifeq ($(OS),Windows_NT)
	cmd /C rmdir /Q /S $(BUILD_DIR)
else
	-rm -fR $(BUILD_DIR)
endif

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***