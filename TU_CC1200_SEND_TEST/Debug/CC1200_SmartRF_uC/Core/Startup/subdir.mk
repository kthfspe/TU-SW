################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../CC1200_SmartRF_uC/Core/Startup/startup_stm32f407vgtx.s 

OBJS += \
./CC1200_SmartRF_uC/Core/Startup/startup_stm32f407vgtx.o 

S_DEPS += \
./CC1200_SmartRF_uC/Core/Startup/startup_stm32f407vgtx.d 


# Each subdirectory must supply rules for building sources it contributes
CC1200_SmartRF_uC/Core/Startup/%.o: ../CC1200_SmartRF_uC/Core/Startup/%.s CC1200_SmartRF_uC/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-CC1200_SmartRF_uC-2f-Core-2f-Startup

clean-CC1200_SmartRF_uC-2f-Core-2f-Startup:
	-$(RM) ./CC1200_SmartRF_uC/Core/Startup/startup_stm32f407vgtx.d ./CC1200_SmartRF_uC/Core/Startup/startup_stm32f407vgtx.o

.PHONY: clean-CC1200_SmartRF_uC-2f-Core-2f-Startup

