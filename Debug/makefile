################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: 3DGraphicsEngineProject.axf

# Tool invocations
3DGraphicsEngineProject.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/Users/archie/Documents/MCUXpressoIDE_11.3.0/workspace/CMSIS_CORE_LPC17xx/Debug" -Xlinker -Map="3DGraphicsEngineProject.map" -Xlinker --cref -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "3DGraphicsEngineProject_Debug.ld" -o "3DGraphicsEngineProject.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) 3DGraphicsEngineProject.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "3DGraphicsEngineProject.axf"; # arm-none-eabi-objcopy -v -O binary "3DGraphicsEngineProject.axf" "3DGraphicsEngineProject.bin" ; # checksum -p LPC1769 -d "3DGraphicsEngineProject.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
