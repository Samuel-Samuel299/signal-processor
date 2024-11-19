################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
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
E-design\ project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
E-design\ project.list \

OBJCOPY_BIN += \
E-design\ project.bin \


# All Target
all: E-design\ project.elf secondary-outputs

# Tool invocations
E-design\ project.elf: $(OBJS) $(USER_OBJS) C:\Users\23594780\Desktop\E-design\ progress\0600\E-design\ project\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "E-design project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\23594780\Desktop\E-design progress\0600\E-design project\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="E-design project.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

E-design\ project.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "E-design project.list"
	@echo 'Finished building: $@'
	@echo ' '

E-design\ project.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "E-design project.bin"
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
