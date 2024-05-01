################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include stm32_F103_C6_Drivers/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include HAL/subdir.mk
-include APP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
GP_Collision_Avoidance.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
GP_Collision_Avoidance.list \

OBJCOPY_BIN += \
GP_Collision_Avoidance.bin \


# All Target
all: GP_Collision_Avoidance.elf secondary-outputs

# Tool invocations
GP_Collision_Avoidance.elf: $(OBJS) $(USER_OBJS) D:\Embedded\ Systems\Keroles\ projects\GP_Collision_Avoidance\STM32F103C6TX_FLASH.ld
	arm-none-eabi-gcc -o "GP_Collision_Avoidance.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\Embedded Systems\Keroles projects\GP_Collision_Avoidance\STM32F103C6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="GP_Collision_Avoidance.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

GP_Collision_Avoidance.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "GP_Collision_Avoidance.list"
	@echo 'Finished building: $@'
	@echo ' '

GP_Collision_Avoidance.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "GP_Collision_Avoidance.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
