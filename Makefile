################################################################################
# File automatically generated by tool: [projectgenerator]
# version: [2.27.0]
# date: [Wed Apr 04 00:04:33 EDT 2018] 
################################################################################


TARGET = filter-benchmarks


# debug build?
DEBUG = 1
# optimization
OPT = -Og


# source path
SOURCES_DIR = Drivers/CMSIS \
	      Application/User \
	      Application/User/Src \
	      Drivers/STM32L4xx_HAL_Driver \
	      Drivers

# firmware library path
PERIFLIB_PATH = /home/zack/STM32Cube/Repository/STM32Cube_FW_L4_V1.11.0/

# Build path
BUILD_DIR = build

# C sources
C_SOURCES = Src/main.c \
	    Src/stm32l4xx_it.c \
	    Src/system_stm32l4xx.c

# ASM sources
ASM_SOURCES = startup_stm32l432xx.s

HAL_SRC_DIR=Drivers/STM32L4xx_HAL_Driver/Src/
DSP_SRC_DIR=Drivers/CMSIS/DSP_Lib/Source/
PERIFLIB_SOURCES = $(HAL_SRC_DIR)/stm32l4xx_ll_exti.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_gpio.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_pwr.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_rcc.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_usart.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_dma.c \
		   $(HAL_SRC_DIR)/stm32l4xx_ll_utils.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_biquad_cascade_df1_f32.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_biquad_cascade_df1_init_f32.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_fir_f32.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_fir_init_f32.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_iir_lattice_f32.c \
		   $(DSP_SRC_DIR)/FilteringFunctions/arm_iir_lattice_init_f32.c


C_SOURCES += $(foreach s, $(PERIFLIB_SOURCES), $(addprefix $(PERIFLIB_PATH), $(s)))


BINPATH = /usr/bin
PREFIX = arm-none-eabi-
CC = $(BINPATH)/$(PREFIX)gcc
AS = $(BINPATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(BINPATH)/$(PREFIX)objcopy
AR = $(BINPATH)/$(PREFIX)ar
SZ = $(BINPATH)/$(PREFIX)size
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
C_DEFS = -DSTM32L432xx \
	 -DUSE_FULL_LL_DRIVER \
	 -DARM_MATH_CM4 \
	 -D__FPU_PRESENT=1


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = -IInc \
	     -I$(PERIFLIB_PATH)/Drivers/STM32L4xx_HAL_Driver/Inc \
	     -I$(PERIFLIB_PATH)/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
	     -I$(PERIFLIB_PATH)/Drivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall \
	  -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall \
	 -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
    CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"


# link script
LDSCRIPT = STM32L432KCUx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) \
	  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex \
    $(BUILD_DIR)/$(TARGET).bin


# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
	vpath %.c $(sort $(dir $(C_SOURCES)))
	# list of ASM program objects
	OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
	vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	

$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR .dep $(BUILD_DIR)

# dependencies
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
