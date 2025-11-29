################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BMI088/Accel.c \
../Drivers/BMI088/Gyro.c \
../Drivers/BMI088/IMU.c \
../Drivers/BMI088/Vectors.c 

OBJS += \
./Drivers/BMI088/Accel.o \
./Drivers/BMI088/Gyro.o \
./Drivers/BMI088/IMU.o \
./Drivers/BMI088/Vectors.o 

C_DEPS += \
./Drivers/BMI088/Accel.d \
./Drivers/BMI088/Gyro.d \
./Drivers/BMI088/IMU.d \
./Drivers/BMI088/Vectors.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BMI088/%.o Drivers/BMI088/%.su Drivers/BMI088/%.cyclo: ../Drivers/BMI088/%.c Drivers/BMI088/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U545xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Cindy/Downloads/Droid/Droid/Drivers/BMI088" -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM33_NTZ/non_secure/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BMI088

clean-Drivers-2f-BMI088:
	-$(RM) ./Drivers/BMI088/Accel.cyclo ./Drivers/BMI088/Accel.d ./Drivers/BMI088/Accel.o ./Drivers/BMI088/Accel.su ./Drivers/BMI088/Gyro.cyclo ./Drivers/BMI088/Gyro.d ./Drivers/BMI088/Gyro.o ./Drivers/BMI088/Gyro.su ./Drivers/BMI088/IMU.cyclo ./Drivers/BMI088/IMU.d ./Drivers/BMI088/IMU.o ./Drivers/BMI088/IMU.su ./Drivers/BMI088/Vectors.cyclo ./Drivers/BMI088/Vectors.d ./Drivers/BMI088/Vectors.o ./Drivers/BMI088/Vectors.su

.PHONY: clean-Drivers-2f-BMI088

