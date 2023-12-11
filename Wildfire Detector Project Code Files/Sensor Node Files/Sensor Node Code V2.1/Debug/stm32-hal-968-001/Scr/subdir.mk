################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32-hal-968-001/Scr/ULPSM-CO-968-001.c 

OBJS += \
./stm32-hal-968-001/Scr/ULPSM-CO-968-001.o 

C_DEPS += \
./stm32-hal-968-001/Scr/ULPSM-CO-968-001.d 


# Each subdirectory must supply rules for building sources it contributes
stm32-hal-968-001/Scr/%.o stm32-hal-968-001/Scr/%.su: ../stm32-hal-968-001/Scr/%.c stm32-hal-968-001/Scr/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/lucia/STM32CubeIDE/workspace_1.9.0/Nodo_Sensor_TesisV2/stm32-hal-rfm95/Inc" -I"C:/Users/lucia/STM32CubeIDE/workspace_1.9.0/Nodo_Sensor_TesisV2/stm32-hal-sht3x-master/Inc" -I"C:/Users/lucia/STM32CubeIDE/workspace_1.9.0/Nodo_Sensor_TesisV2/stm32-hal-968-001/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32-2d-hal-2d-968-2d-001-2f-Scr

clean-stm32-2d-hal-2d-968-2d-001-2f-Scr:
	-$(RM) ./stm32-hal-968-001/Scr/ULPSM-CO-968-001.d ./stm32-hal-968-001/Scr/ULPSM-CO-968-001.o ./stm32-hal-968-001/Scr/ULPSM-CO-968-001.su

.PHONY: clean-stm32-2d-hal-2d-968-2d-001-2f-Scr

