################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../GyroKalman.cpp \
../HMC5883.cpp \
../IMU_Kalman.cpp \
../MPU6000.cpp 

OBJS += \
./GyroKalman.o \
./HMC5883.o \
./IMU_Kalman.o \
./MPU6000.o 

CPP_DEPS += \
./GyroKalman.d \
./HMC5883.d \
./IMU_Kalman.d \
./MPU6000.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\cores\arduino" -I"C:\arduino-1.0.4-windows\arduino-1.0.4\hardware\arduino\variants\standard" -I"D:\Documents\Arduino\IMU_Kalman" -I"D:\Documents\Arduino\IMU_Kalman\avrfix" -I"D:\Documents\Arduino\IMU_Kalman\SPI" -I"D:\Documents\Arduino\IMU_Kalman\Wire" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes" -I"D:\Documents\Arduino\IMU_Kalman\avr_includes\avr" -D__IN_ECLIPSE__=1 -DUSB_VID= -DUSB_PID= -DARDUINO=104 -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '


