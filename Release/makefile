################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Wire/subdir.mk
-include SPI/subdir.mk
-include MPU6000/subdir.mk
-include Kalman_Filters/subdir.mk
-include HMC5883/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
IMU_Kalman.lss \

FLASH_IMAGE += \
IMU_Kalman.hex \

EEPROM_IMAGE += \
IMU_Kalman.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: IMU_Kalman.elf secondary-outputs

# Tool invocations
IMU_Kalman.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-gcc -Os -Wl,--gc-sections  -Wl,-Map,"IMU_Kalman.map",--cref --export-all-symbols -L"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\avrfix" -L"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\Board_Support_Library" -mmcu=atmega328p  -o"IMU_Kalman.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '

IMU_Kalman.lss: IMU_Kalman.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S IMU_Kalman.elf  >"IMU_Kalman.lss"
	@echo 'Finished building: $@'
	@echo ' '

IMU_Kalman.hex: IMU_Kalman.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex IMU_Kalman.elf  "IMU_Kalman.hex"
	@echo 'Finished building: $@'
	@echo ' '

IMU_Kalman.eep: IMU_Kalman.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex IMU_Kalman.elf  "IMU_Kalman.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: IMU_Kalman.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p IMU_Kalman.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(INO_DEPS)$(CPP_DEPS)$(PDE_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) IMU_Kalman.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
