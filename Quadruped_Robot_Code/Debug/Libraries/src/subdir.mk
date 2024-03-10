################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/src/MPU6050.c \
../Libraries/src/pca9685.c 

OBJS += \
./Libraries/src/MPU6050.o \
./Libraries/src/pca9685.o 

C_DEPS += \
./Libraries/src/MPU6050.d \
./Libraries/src/pca9685.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/src/%.o Libraries/src/%.su Libraries/src/%.cyclo: ../Libraries/src/%.c Libraries/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mouni/STM32CubeIDE/RVCE_EL_Project/Quadruped-Robot/Quadruped_Robot_Code/Libraries/inc" -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libraries-2f-src

clean-Libraries-2f-src:
	-$(RM) ./Libraries/src/MPU6050.cyclo ./Libraries/src/MPU6050.d ./Libraries/src/MPU6050.o ./Libraries/src/MPU6050.su ./Libraries/src/pca9685.cyclo ./Libraries/src/pca9685.d ./Libraries/src/pca9685.o ./Libraries/src/pca9685.su

.PHONY: clean-Libraries-2f-src

