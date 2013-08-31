################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Board_Support_Library/Arduino/CDC.cpp \
../Board_Support_Library/Arduino/HID.cpp \
../Board_Support_Library/Arduino/HardwareSerial.cpp \
../Board_Support_Library/Arduino/IPAddress.cpp \
../Board_Support_Library/Arduino/Print.cpp \
../Board_Support_Library/Arduino/Stream.cpp \
../Board_Support_Library/Arduino/Tone.cpp \
../Board_Support_Library/Arduino/USBCore.cpp \
../Board_Support_Library/Arduino/WMath.cpp \
../Board_Support_Library/Arduino/WString.cpp \
../Board_Support_Library/Arduino/main.cpp \
../Board_Support_Library/Arduino/new.cpp 

C_SRCS += \
../Board_Support_Library/Arduino/WInterrupts.c \
../Board_Support_Library/Arduino/malloc.c \
../Board_Support_Library/Arduino/wiring.c \
../Board_Support_Library/Arduino/wiring_analog.c \
../Board_Support_Library/Arduino/wiring_digital.c \
../Board_Support_Library/Arduino/wiring_pulse.c \
../Board_Support_Library/Arduino/wiring_shift.c 

OBJS += \
./Board_Support_Library/Arduino/CDC.o \
./Board_Support_Library/Arduino/HID.o \
./Board_Support_Library/Arduino/HardwareSerial.o \
./Board_Support_Library/Arduino/IPAddress.o \
./Board_Support_Library/Arduino/Print.o \
./Board_Support_Library/Arduino/Stream.o \
./Board_Support_Library/Arduino/Tone.o \
./Board_Support_Library/Arduino/USBCore.o \
./Board_Support_Library/Arduino/WInterrupts.o \
./Board_Support_Library/Arduino/WMath.o \
./Board_Support_Library/Arduino/WString.o \
./Board_Support_Library/Arduino/main.o \
./Board_Support_Library/Arduino/malloc.o \
./Board_Support_Library/Arduino/new.o \
./Board_Support_Library/Arduino/wiring.o \
./Board_Support_Library/Arduino/wiring_analog.o \
./Board_Support_Library/Arduino/wiring_digital.o \
./Board_Support_Library/Arduino/wiring_pulse.o \
./Board_Support_Library/Arduino/wiring_shift.o 

C_DEPS += \
./Board_Support_Library/Arduino/WInterrupts.d \
./Board_Support_Library/Arduino/malloc.d \
./Board_Support_Library/Arduino/wiring.d \
./Board_Support_Library/Arduino/wiring_analog.d \
./Board_Support_Library/Arduino/wiring_digital.d \
./Board_Support_Library/Arduino/wiring_pulse.d \
./Board_Support_Library/Arduino/wiring_shift.d 

CPP_DEPS += \
./Board_Support_Library/Arduino/CDC.d \
./Board_Support_Library/Arduino/HID.d \
./Board_Support_Library/Arduino/HardwareSerial.d \
./Board_Support_Library/Arduino/IPAddress.d \
./Board_Support_Library/Arduino/Print.d \
./Board_Support_Library/Arduino/Stream.d \
./Board_Support_Library/Arduino/Tone.d \
./Board_Support_Library/Arduino/USBCore.d \
./Board_Support_Library/Arduino/WMath.d \
./Board_Support_Library/Arduino/WString.d \
./Board_Support_Library/Arduino/main.d \
./Board_Support_Library/Arduino/new.d 


# Each subdirectory must supply rules for building sources it contributes
Board_Support_Library/Arduino/%.o: ../Board_Support_Library/Arduino/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"D:\Documents\Arduino\IMU_Kalman\Board_Support_Library\Arduino" -I"D:\Documents\Arduino\IMU_Kalman\Board_Support_Library\standard" -I"D:\Documents\Arduino\IMU_Kalman" -I"D:\Documents\Arduino\IMU_Kalman\avrfix" -I"D:\Documents\Arduino\IMU_Kalman\SPI" -I"D:\Documents\Arduino\IMU_Kalman\Wire" -I"D:\Documents\Arduino\IMU_Kalman\HMC5883" -I"D:\Documents\Arduino\IMU_Kalman\MPU6000" -I"D:\Documents\Arduino\IMU_Kalman\Kalman_Filters" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes\avr" -D__IN_ECLIPSE__=1 -DUSB_VID= -DUSB_PID= -DARDUINO=104 -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '

Board_Support_Library/Arduino/%.o: ../Board_Support_Library/Arduino/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"D:\Documents\Arduino\IMU_Kalman\Board_Support_Library\Arduino" -I"D:\Documents\Arduino\IMU_Kalman\Board_Support_Library\standard" -I"D:\Documents\Arduino\IMU_Kalman" -I"D:\Documents\Arduino\IMU_Kalman\avrfix" -I"D:\Documents\Arduino\IMU_Kalman\SPI" -I"D:\Documents\Arduino\IMU_Kalman\Wire" -I"D:\Documents\Arduino\IMU_Kalman\HMC5883" -I"D:\Documents\Arduino\IMU_Kalman\MPU6000" -I"D:\Documents\Arduino\IMU_Kalman\Kalman_Filters" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes\avr" -D__IN_ECLIPSE__=1 -DARDUINO=104 -DUSB_PID= -DUSB_VID= -Wall -Os -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


