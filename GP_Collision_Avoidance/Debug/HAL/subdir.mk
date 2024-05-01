################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../HAL/Ultrasonic_Sensor.c 

OBJS += \
./HAL/Ultrasonic_Sensor.o 

C_DEPS += \
./HAL/Ultrasonic_Sensor.d 


# Each subdirectory must supply rules for building sources it contributes
HAL/Ultrasonic_Sensor.o: ../HAL/Ultrasonic_Sensor.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"HAL/Ultrasonic_Sensor.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

