################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c 

OBJS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.o \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.o 

C_DEPS += \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.d \
./Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_cortex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_dma_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_flash_ramfunc.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_gpio.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_i2c_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_pwr_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_rcc_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Drivers/STM32F4xx_HAL_Driver/stm32f4xx_hal_tim_ex.o: D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F411xE -I"D:/ARM_projects/06_I2C/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc" -I"D:/ARM_projects/06_I2C/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"D:/ARM_projects/06_I2C/Drivers/CMSIS/Include"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


