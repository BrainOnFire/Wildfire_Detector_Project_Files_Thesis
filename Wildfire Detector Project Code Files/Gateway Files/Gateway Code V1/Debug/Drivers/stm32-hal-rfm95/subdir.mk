################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/stm32-hal-rfm95/rfm95.c 

OBJS += \
./Drivers/stm32-hal-rfm95/rfm95.o 

C_DEPS += \
./Drivers/stm32-hal-rfm95/rfm95.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/stm32-hal-rfm95/%.o Drivers/stm32-hal-rfm95/%.su: ../Drivers/stm32-hal-rfm95/%.c Drivers/stm32-hal-rfm95/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Drivers/stm32-hal-rfm95 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-stm32-2d-hal-2d-rfm95

clean-Drivers-2f-stm32-2d-hal-2d-rfm95:
	-$(RM) ./Drivers/stm32-hal-rfm95/rfm95.d ./Drivers/stm32-hal-rfm95/rfm95.o ./Drivers/stm32-hal-rfm95/rfm95.su

.PHONY: clean-Drivers-2f-stm32-2d-hal-2d-rfm95

