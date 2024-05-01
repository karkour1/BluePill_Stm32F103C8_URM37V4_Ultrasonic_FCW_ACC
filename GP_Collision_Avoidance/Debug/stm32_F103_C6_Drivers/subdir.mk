################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.c \
../stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.c \
../stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.c \
../stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.c \
../stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.c 

OBJS += \
./stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.o \
./stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.o \
./stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.o \
./stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.o \
./stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.o 

C_DEPS += \
./stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.d \
./stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.d \
./stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.d \
./stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.d \
./stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.o: ../stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32_F103_C6_Drivers/Stm32_F103C6_EXTI_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.o: ../stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32_F103_C6_Drivers/Stm32_F103C6_RCC_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.o: ../stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32_F103_C6_Drivers/Stm32_F103C6_TIMERS_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.o: ../stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32_F103_C6_Drivers/Stm32_F103C6_UART_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.o: ../stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DSTM32 -DSTM32F1 -DSTM32F103C6Tx -DDEBUG -c -I../Inc -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/HAL/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/stm32_F103_C6_Drivers/inc" -I"D:/Embedded Systems/Keroles projects/GP_Collision_Avoidance/APP/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"stm32_F103_C6_Drivers/Stm32_F103C6_gpio_driver.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

