PROJECT_NAME     := nucleo-l412kb_dev

#################################################
#                  T A R G E T
#################################################
TARGET = nucleo-l412kb_dev



#################################################
#     B U I L D I N G    V A R I A B L E S
#################################################
# ## debug build?
DEBUG = 1
# ## Optimization
#OPT = -Os
#OPT = -Og
OPT =


#################################################
#                    P A T H S
#################################################
### Build path
BUILD_DIR = _build
### Output directory
OUTPUT_DIRECTORY := _build



#################################################
#            S O U R C E    F I L E S
#################################################
# ASSEMBLY SOURCE FILES
ASM_SOURCES = Core/Startup/startup_stm32l412kbux.s

# C SOURCE FILES
C_SOURCES =  \
Core/Src/main.c \
Core/Src/stm32l4xx_it.c \
Core/Src/syscalls.c \
Core/Src/sysmem.c \
Core/Src/system_stm32l4xx.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_pwr.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \



#################################################
#           I N C L U D E    F I L E S
#################################################
## ASSEMBLY INCLUDES
AS_INCLUDES = 

## C INCLUDES
C_INCLUDES =  \
 -IConfig/Inc \
 -ICore/Inc \
 -IDrivers/CMSIS/Device/ST/STM32L4xx/Include \
 -IDrivers/CMSIS/Include \
 -IDrivers/STM32L4xx_HAL_Driver/Inc \

 

#################################################
#                B I N A R I E S
#################################################
PREFIX = arm-none-eabi-
### The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
### either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S



#################################################
#               D E F I N E S
#################################################
## ASSEMBLY DEFINES
AS_DEFS = 

## C DEFINES
C_DEFS += -DUSE_FULL_LL_DRIVER 
C_DEFS += -DUSE_FULL_ASSERT 
C_DEFS += -DHSE_VALUE=8000000 # Value of the External oscillator in Hz. HSE_VALUE is defined in system_stm32l4xx.c and stm32l4xx_ll_rcc.h (and stm32l4xx_hal_conf.h)
C_DEFS += -DLSE_VALUE=32768 # Value of the LSE oscillator in Hz. Defined in stm32l4xx_ll_rcc.h (and stm32l4xx_hal_conf.h)
C_DEFS += -DMSI_VALUE=4000000 # Defined in system_stm32l4xx.c (and stm32l4xx_hal_conf.h)
C_DEFS += -DHSI_VALUE=16000000 # Defined in system_stm32l4xx.c and stm32l4xx_ll_rcc.h (and stm32l4xx_hal_conf.h)
C_DEFS += -DLSI_VALUE=32000 # Defined in stm32l4xx_ll_rcc.h
C_DEFS += -DARM_MATH_CM4 
C_DEFS += -D__BYTE_ORDER__=__ORDER_LITTLE_ENDIAN__ 
C_DEFS += -DSTM32L412xx



#################################################
#                 F L A G S
#################################################
## COMMON MCU SPECIFICATION FLAG
CPU = -mcpu=cortex-m4 # cpu
FPU = -mfpu=fpv4-sp-d16 # fpu
FLOAT-ABI = -mfloat-abi=hard # float-abi
MCU = $(CPU) -mthumb $(FLOAT-ABI) $(FPU) # mcu

## C FLAGS
CFLAGS += $(MCU)
CFLAGS += $(C_DEFS)
CFLAGS += $(C_INCLUDES) 
CFLAGS += $(OPT)
CFLAGS += -std=gnu11
ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2 
endif
CFLAGS += -Wall 
CFLAGS += -Wextra 
### Don't escalate unused functions and variables to errors..  
CFLAGS += -Wno-error=unused-function -Wno-error=unused-variable -Wmissing-include-dirs -Wswitch-default -fshort-enums -specs=nano.specs
CFLAGS += -fstack-usage 
### Don't escalate code monkey generated #warning's to errors  
CFLAGS += -Wno-error=cpp
### Keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections


### Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

## ASSEMBLY FLAGS
ASFLAGS += $(MCU) 
ASFLAGS += $(AS_DEFS) 
ASFLAGS += $(AS_INCLUDES) 
ASFLAGS += $(OPT) 
ASFLAGS += -Wall 
ASFLAGS += -fdata-sections -ffunction-sections


## LINKER SCRIPT FLAGS
LDSCRIPT = STM32L412KBUX_FLASH.ld
### Libraries
LIBS = -lc -lm -lnosys
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections


# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin



#################################################
#        B U I L D    A P P L I C A T I O N
#################################################
## LIST OF C OBJECTS
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
## LIST OF ASM PROGRAM OBJECTS
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



#################################################
#             C L E A N    U P
#################################################
clean:
	-rm -fR $(BUILD_DIR)/*



#################################################
#            D E P E N D E N C I E S
#################################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***