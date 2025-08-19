TARGET = stm32g4_pro

CHIP   = STM32G474RE

# debug build?
DEBUG = 1
# optimization
# OPT = -Og
OPT = -O3 -g3

# Build path
BUILD_DIR = build
SDK_DIR   = ../dh_sdk

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-

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
C_DEFS =  			\
-DUSE_HAL_DRIVER 	\
-DSTM32G474xx		\
-DTRACE_ENABLE		\
-DTRACE_LEVEL=7		\


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  		\
-I user		 		\
-I user/inc 		\
-I $(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Inc 			\
-I $(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Inc/Legacy 	\
-I $(SDK_DIR)/platform/stm32/stm32g4/CMSIS/Device/ST/STM32G4xx/Include 	\
-I $(SDK_DIR)/platform/stm32/stm32g4/CMSIS/Include						\
-I $(SDK_DIR)/components/trace	\
-I peripherals								\
-I bsp										\
-I peripherals								\

# C sources
C_SOURCES =  	\
user/main.c 	\
user/src/stm32g4xx_it.c 		\
user/src/stm32g4xx_hal_msp.c 	\
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
$(SDK_DIR)/platform/stm32/stm32g4/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
$(SDK_DIR)/components/trace/trace.c								\
user/Src/system_stm32g4xx.c 		\
user/Src/sysmem.c 		\
user/Src/syscalls.c  	\
peripherals/uart.c		\
peripherals/gpio.c		\
peripherals/delay.c		\
peripherals/sys.c		\
peripherals/retarget.c	\
peripherals/timer.c		\
peripherals/adc.c		\
user/parameters.c		\

# ASM sources
ASM_SOURCES =  \
startup_stm32g474xx.s

# ASMM sources
ASMM_SOURCES = 

# ASMMC sources
ASMMC_SOURCE = 


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32G474XX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin flash_app_jlink erase_jlink reset_jlink

flash_app_jlink: $(BUILD_DIR)/$(TARGET)_app.jlink
erase_jlink: $(BUILD_DIR)/erase.jlink
reset_jlink: $(BUILD_DIR)/reset.jlink


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMMC_SOURCES:.c=.o)))
vpath %.S $(sort $(dir $(ASMMC_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
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

# 烧写app flash
$(BUILD_DIR)/$(TARGET)_app.jlink: $(BUILD_DIR)/$(TARGET).hex
	@echo "device $(CHIP)" > $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "if swd" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "speed 500" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "r" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "sleep 100" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "loadfile $(BUILD_DIR)/$(TARGET).hex" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "r" >> $(BUILD_DIR)/$(TARGET)_app.jlink
	@echo "exit" >> $(BUILD_DIR)/$(TARGET)_app.jlink

# 烧录app
flash:
	JLink -commandfile $(BUILD_DIR)/$(TARGET)_app.jlink

# 擦除
$(BUILD_DIR)/erase.jlink:
	@echo "device $(CHIP)" > $(BUILD_DIR)/erase.jlink
	@echo "if swd" >> $(BUILD_DIR)/erase.jlink
	@echo "speed 500" >> $(BUILD_DIR)/erase.jlink
	@echo "r" >> $(BUILD_DIR)/erase.jlink
	@echo "sleep 100" >> $(BUILD_DIR)/erase.jlink
	@echo "erase" >> $(BUILD_DIR)/erase.jlink
	@echo "r" >> $(BUILD_DIR)/erase.jlink
	@echo "exit" >> $(BUILD_DIR)/erase.jlink

# 擦除
$(BUILD_DIR)/reset.jlink:
	@echo "device $(CHIP)" > $(BUILD_DIR)/reset.jlink
	@echo "if swd" >> $(BUILD_DIR)/reset.jlink
	@echo "speed 500" >> $(BUILD_DIR)/reset.jlink
	@echo "r" >> $(BUILD_DIR)/reset.jlink
	@echo "exit" >> $(BUILD_DIR)/reset.jlink

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***