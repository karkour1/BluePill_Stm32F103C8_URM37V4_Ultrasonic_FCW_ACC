################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/Collision_Avoidance.c 

OBJS += \
./APP/Collision_Avoidance.o 

C_DEPS += \
./APP/Collision_Avoidance.d 


# Each subdirectory must supply rules for building sources it contributes
APP/Collision_Avoidance.o: ../APP/Collision_Avoidance.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"APP/Collision_Avoidance.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

