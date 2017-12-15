################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/crc.c \
../Src/gpio.c \
../Src/main.c \
../Src/stm32f7xx_hal_msp.c \
../Src/stm32f7xx_it.c \
../Src/system_stm32f7xx.c \
../Src/usart.c 

OBJS += \
./Src/crc.o \
./Src/gpio.o \
./Src/main.o \
./Src/stm32f7xx_hal_msp.o \
./Src/stm32f7xx_it.o \
./Src/system_stm32f7xx.o \
./Src/usart.o 

C_DEPS += \
./Src/crc.d \
./Src/gpio.d \
./Src/main.d \
./Src/stm32f7xx_hal_msp.d \
./Src/stm32f7xx_it.d \
./Src/system_stm32f7xx.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 '-D__weak=__attribute__((weak))' '-D__packed="__attribute__((__packed__))"' -DUSE_HAL_DRIVER -DSTM32F767xx -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Inc" -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Drivers/STM32F7xx_HAL_Driver/Inc" -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Drivers/CMSIS/Include" -I"A:/Google_Drive/Programowanie/ARM_projects/f7_uart_test/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


