################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32-hal-rfm95-master/rfm95.c 

OBJS += \
./stm32-hal-rfm95-master/rfm95.o 

C_DEPS += \
./stm32-hal-rfm95-master/rfm95.d 


# Each subdirectory must supply rules for building sources it contributes
stm32-hal-rfm95-master/rfm95.o: C:/Users/lucia/Documents/STM32Cube_Projects/Nodo_Sensor_Tesis/stm32-hal-rfm95-master/rfm95.c stm32-hal-rfm95-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/lucia/Documents/STM32Cube_Projects/Nodo_Sensor_Tesis/stm32-hal-rfm95-master" -I"C:/Users/lucia/Documents/STM32Cube_Projects/Nodo_Sensor_Tesis/stm32-hal-sht3x-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32-2d-hal-2d-rfm95-2d-master

clean-stm32-2d-hal-2d-rfm95-2d-master:
	-$(RM) ./stm32-hal-rfm95-master/rfm95.d ./stm32-hal-rfm95-master/rfm95.o ./stm32-hal-rfm95-master/rfm95.su

.PHONY: clean-stm32-2d-hal-2d-rfm95-2d-master

