################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
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
Recieve_test.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Recieve_test.list \

OBJCOPY_BIN += \
Recieve_test.bin \


# All Target
all: Recieve_test.elf secondary-outputs

# Tool invocations
Recieve_test.elf: $(OBJS) $(USER_OBJS) C:\Users\jocke\STM32CubeIDE\workspace_1.2.0\Recieve_test\STM32F091RCTX_FLASH.ld
	arm-none-eabi-gcc -o "Recieve_test.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\jocke\STM32CubeIDE\workspace_1.2.0\Recieve_test\STM32F091RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Recieve_test.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Recieve_test.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Recieve_test.list"
	@echo 'Finished building: $@'
	@echo ' '

Recieve_test.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Recieve_test.bin"
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
