################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../SPI/SPI.cpp 

OBJS += \
./SPI/SPI.o 

CPP_DEPS += \
./SPI/SPI.d 


# Each subdirectory must supply rules for building sources it contributes
SPI/%.o: ../SPI/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\Board_Support_Library\Arduino" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\Board_Support_Library\standard" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\avrfix" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\SPI" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\Wire" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\HMC5883" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\MPU6000" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\Kalman_Filters" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\avr_includes" -I"D:\Documents\QuadCopter\Code\IMU\KalmanIMU\avr_includes\avr" -D__IN_ECLIPSE__=1 -DUSB_VID= -DUSB_PID= -DARDUINO=104 -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -g -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)"  -c -o "$@" -x c++ "$<"
	@echo 'Finished building: $<'
	@echo ' '


