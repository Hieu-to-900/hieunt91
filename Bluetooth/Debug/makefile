################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Bluetooth.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Bluetooth.list \


# All Target
all: Bluetooth.elf secondary-outputs

# Tool invocations
Bluetooth.elf: $(OBJS) $(USER_OBJS) C:\Users\HP\ Desktop\STM32CubeIDE\workspace_1.1.0\Bluetooth\STM32G071RBTX_FLASH.ld
	arm-none-eabi-gcc -o "Bluetooth.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"C:\Users\HP Desktop\STM32CubeIDE\workspace_1.1.0\Bluetooth\STM32G071RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Bluetooth.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Bluetooth.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Bluetooth.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
