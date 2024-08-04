######################################
# target
######################################
TARGET = wlb

BOARD = typec

BOARD_BASE = lib/${BOARD}-board-base

ifeq ($(BOARD), typec)
	STARTUP_POSTFIX = stm32f407xx
	LINK_SCRIPT_PREFIX = STM32F407IGHx
	BOARD_C_DEF = STM32F407xx
endif

ifeq ($(BOARD), typea)
	STARTUP_POSTFIX = stm32f407xx
	LINK_SCRIPT_PREFIX = STM32F407IGHx
	BOARD_C_DEF = STM32F407xx
endif
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
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
$(BOARD_BASE)/Core/Src/main.c \
$(BOARD_BASE)/Core/Src/gpio.c \
$(BOARD_BASE)/Core/Src/freertos.c \
$(BOARD_BASE)/Core/Src/can.c \
$(BOARD_BASE)/Core/Src/dma.c \
$(BOARD_BASE)/Core/Src/i2c.c \
$(BOARD_BASE)/Core/Src/spi.c \
$(BOARD_BASE)/Core/Src/tim.c \
$(BOARD_BASE)/Core/Src/usart.c \
$(BOARD_BASE)/Core/Src/stm32f4xx_it.c \
$(BOARD_BASE)/Core/Src/stm32f4xx_hal_msp.c \
$(BOARD_BASE)/Core/Src/stm32f4xx_hal_timebase_tim.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
$(BOARD_BASE)/Core/Src/system_stm32f4xx.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/list.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
lib/CMSIS-DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
lib/CMSIS-DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c \
lib/CMSIS-DSP/Source/BayesFunctions/BayesFunctions.c \
lib/CMSIS-DSP/Source/CommonTables/CommonTables.c \
lib/CMSIS-DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
lib/CMSIS-DSP/Source/ControllerFunctions/ControllerFunctions.c \
lib/CMSIS-DSP/Source/DistanceFunctions/DistanceFunctions.c \
lib/CMSIS-DSP/Source/FastMathFunctions/FastMathFunctions.c \
lib/CMSIS-DSP/Source/FilteringFunctions/FilteringFunctions.c \
lib/CMSIS-DSP/Source/MatrixFunctions/MatrixFunctions.c \
lib/CMSIS-DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
lib/CMSIS-DSP/Source/SupportFunctions/SupportFunctions.c \
lib/CMSIS-DSP/Source/SVMFunctions/SVMFunctions.c \
lib/CMSIS-DSP/Source/TransformFunctions/TransformFunctions.c \
lib/CMSIS-DSP/Source/InterpolationFunctions/InterpolationFunctions.c \
lib/CMSIS-DSP/Source/BasicMathFunctions/BasicMathFunctionsF16.c \
lib/CMSIS-DSP/Source/ComplexMathFunctions/ComplexMathFunctionsF16.c \
lib/CMSIS-DSP/Source/FilteringFunctions/FilteringFunctionsF16.c \
lib/CMSIS-DSP/Source/CommonTables/CommonTablesF16.c \
lib/CMSIS-DSP/Source/TransformFunctions/TransformFunctionsF16.c \
lib/CMSIS-DSP/Source/MatrixFunctions/MatrixFunctionsF16.c \
lib/CMSIS-DSP/Source/InterpolationFunctions/InterpolationFunctionsF16.c \
lib/CMSIS-DSP/Source/StatisticsFunctions/StatisticsFunctionsF16.c \
lib/CMSIS-DSP/Source/SupportFunctions/SupportFunctionsF16.c \
lib/CMSIS-DSP/Source/FastMathFunctions/FastMathFunctionsF16.c \
lib/CMSIS-DSP/Source/DistanceFunctions/DistanceFunctionsF16.c \
lib/CMSIS-DSP/Source/BayesFunctions/BayesFunctionsF16.c \
lib/CMSIS-DSP/Source/SVMFunctions/SVMFunctionsF16.c \
src/algo/src/MahonyAHRS.c \
src/algo/src/pid.c \
src/algo/src/Swerve_Locomotion.c \
src/algo/src/wheel_legged_locomotion.c \
src/algo/src/five_bar_leg.c \
src/algo/src/kalman_filter.c \
src/algo/src/wlb_lqr_controller.c \
src/algo/src/FusionAhrs.c \
src/algo/src/FusionCompass.c \
src/algo/src/FusionOffset.c \
src/bsp/src/bsp_can.c \
src/bsp/src/bsp_delay.c \
src/bsp/src/bsp_daemon.c \
src/bsp/src/bsp_pwm.c \
src/bsp/src/bsp_spi.c \
src/bsp/src/bsp_serial.c \
src/bsp/src/bsp_crc.c \
src/bsp/src/bsp_uart.c \
src/devices/src/bmi088driver.c \
src/devices/src/bmi088middleware.c \
src/devices/src/buzzer.c \
src/devices/src/dji_motor.c \
src/devices/src/dm_motor.c \
src/devices/src/ist8310driver.c \
src/devices/src/ist8310driver_middleware.c \
src/devices/src/mf_motor.c \
src/devices/src/remote.c \
src/devices/src/imu_task.c \
src/devices/src/referee_system.c \
src/devices/src/jetson_orin.c \
src/devices/src/laser.c \
src/app/src/motor_task.c \
src/app/src/chassis_task.c \
src/app/src/gimbal_task.c \
src/app/src/launch_task.c \
src/app/src/debug_task.c \
src/app/src/robot.c \
src/app/src/ui_task.c \
src/app/src/board_comm_task.c \
src/ui/src/ui_interface.c \
src/ui/src/ui_indicator_0_0.c \
src/ui/src/ui_indicator_0_1.c \
src/ui/src/ui_indicator_0_2.c \
src/ui/src/ui_indicator_0_3.c \
src/ui/src/ui_indicator_0_4.c \
src/ui/src/ui_indicator_0_5.c \
src/ui/src/ui_indicator_0_6.c \
src/ui/src/ui_indicator_0_7.c \
src/ui/src/ui_indicator_0_8.c \
src/ui/src/ui_indicator_0_9.c \
src/ui/src/ui_indicator_0_10.c \
src/ui/src/ui_indicator_0_11.c \
src/ui/src/ui_indicator_1_0.c \
src/ui/src/ui_indicator_1_1.c 

# ASM sources
ASM_SOURCES =  \
$(BOARD_BASE)/startup_stm32f407xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
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
MCU = $(CPU) -mthumb -mthumb-interwork $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-D$(BOARD_C_DEF)

# AS includes
AS_INCLUDES =  \
-I$(BOARD_BASE)/Core/Inc

# C includes
C_INCLUDES =  \
-I$(BOARD_BASE)/Core/Inc \
-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc \
-I$(BOARD_BASE)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/include \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-I$(BOARD_BASE)/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I$(BOARD_BASE)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I$(BOARD_BASE)/Drivers/CMSIS/Include \
-Ilib/CMSIS-DSP/Include \
-Ilib/CMSIS-DSP/PrivateInclude \
-Isrc/algo/inc \
-Isrc/devices/inc \
-Isrc/bsp/inc \
-Isrc/app/inc \
-Isrc/ui/inc 

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -fmessage-length=0 -Werror

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = $(BOARD_BASE)/$(LINK_SCRIPT_PREFIX)_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -flto -Wl,--print-memory-usage -u _printf_float

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	@$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	@mkdir $@		

#######################################
# clean up
#######################################
clean:
	rm -rf $(BUILD_DIR) /s/q

clean_unix:
	rm -rf $(BUILD_DIR)
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

#######################################
# download task
#######################################

# Windows (Powershell)
ECHO_WARNING_POWERSHELL=powershell Write-Host -ForegroundColor Yellow [Message from Jason]:
ECHO_SUCCESS_POWERSHELL=powershell Write-Host -ForegroundColor Green [Success]

download_powershell:
	@echo "Attempting to use CMSIS-DAP..."
	@openocd -f config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS_POWERSHELL) "Successfully programmed the device using CMSIS-DAP.") || \
	($(ECHO_WARNING_POWERSHELL) "Failed to connect using CMSIS-DAP. Attempting to use STLink..." && \
	openocd -f config/openocd_stlink.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS_POWERSHELL) "Successfully programmed the device using STLink.") || \
	($(ECHO_WARNING_POWERSHELL) "Failed to connect using both CMSIS-DAP and STLink. Please check your connections and try again."))


# Unix-Like (Linux, MacOS)
ECHO_WARNING=echo "\033[33m[Warning]\033[0m"
ECHO_SUCCESS=echo "\033[32m[Success]\033[0m"

download:
	@echo "Attempting to use CMSIS-DAP..."
	@openocd -d2 -f config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS) "Successfully programmed the device using CMSIS-DAP.") || \
	($(ECHO_WARNING) "Failed to connect using CMSIS-DAP. Attempting to use STLink..." && \
	openocd -d2 -f config/openocd_stlink.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown && \
	($(ECHO_SUCCESS) "Successfully programmed the device using STLink.") || \
	($(ECHO_WARNING) "Failed to connect using both CMSIS-DAP and STLink. Please check your connections and try again."))

test_download:
	openocd -d4 -f config/openocd_cmsis_dap.cfg -c init -c halt -c "program $(BUILD_DIR)/$(TARGET).bin 0x08000000 verify reset" -c "reset run" -c shutdown

# *** EOF ***
