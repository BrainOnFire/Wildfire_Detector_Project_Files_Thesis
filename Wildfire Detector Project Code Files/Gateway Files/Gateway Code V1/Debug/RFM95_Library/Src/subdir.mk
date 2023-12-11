################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RFM95_Library/Src/rfm95.c 

OBJS += \
./RFM95_Library/Src/rfm95.o 

C_DEPS += \
./RFM95_Library/Src/rfm95.d 


# Each subdirectory must supply rules for building sources it contributes
RFM95_Library/Src/%.o RFM95_Library/Src/%.su: ../RFM95_Library/Src/%.c RFM95_Library/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/lucia/Documents/STM32_Projects/Gateway_Tesis/RFM95_Library/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-RFM95_Library-2f-Src

clean-RFM95_Library-2f-Src:
	-$(RM) ./RFM95_Library/Src/rfm95.d ./RFM95_Library/Src/rfm95.o ./RFM95_Library/Src/rfm95.su

.PHONY: clean-RFM95_Library-2f-Src

