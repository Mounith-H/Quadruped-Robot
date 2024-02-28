################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/i2c/i2c.c 

OBJS += \
./Libraries/i2c/i2c.o 

C_DEPS += \
./Libraries/i2c/i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/i2c/%.o Libraries/i2c/%.su Libraries/i2c/%.cyclo: ../Libraries/i2c/%.c Libraries/i2c/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mouni/STM32CubeIDE/RVCE_EL_Project/Quadruped-Robot/Quadruped_Robot_Code/Libraries" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libraries-2f-i2c

clean-Libraries-2f-i2c:
	-$(RM) ./Libraries/i2c/i2c.cyclo ./Libraries/i2c/i2c.d ./Libraries/i2c/i2c.o ./Libraries/i2c/i2c.su

.PHONY: clean-Libraries-2f-i2c

