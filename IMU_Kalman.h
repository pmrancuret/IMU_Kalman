// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef IMU_Kalman_H_
#define IMU_Kalman_H_
#include "Arduino.h"
//add your includes for the project IMU_Kalman here
#include "avrfix.h"
#include "MPU6000.h"
#include "HMC5883.h"
#include "GyroKalman.h"
#include "GlobalVariables.h"

//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

// debug definitions
#define DEBUG_MAGNETOMETER		// defining this variable will cause some basic debugging from the magnetometer

//add your function definitions for the project IMU_Kalman here

#define SERIAL_DATARATE 38400	// serial line datarate, baud

#define FASTLOOP_TIME_US 1000000	// fast loop sample rate, in microseconds.

// offsets for gyroscope
#define GYRO_OFFSET_X	0	// gyroscope x-axis offset (in rad/s * 2^24)
#define GYRO_OFFSET_Y	0	// gyroscope y-axis offset (in rad/s * 2^24)
#define GYRO_OFFSET_Z	0	// gyroscope z-axis offset (in rad/s * 2^24)

// offsets for accelerometer
#define ACCEL_OFFSET_X	0	// accelerometer x-axis offset (in m/s^2 * 2^24)
#define ACCEL_OFFSET_Y	0	// accelerometer y-axis offset (in m/s^2 * 2^24)
#define ACCEL_OFFSET_Z	0	// accelerometer z-axis offset (in m/s^2 * 2^24)

// offsets and declination for magnetometer
#define MAG_OFFSET_X	0	// magnetic sensor x-axis offset (in Gauss * 2^24)
#define MAG_OFFSET_Y	0	// magnetic sensor y-axis offset (in Gauss * 2^24)
#define MAG_OFFSET_Z	0	// magnetic sensor z-axis offset (in Gauss * 2^24)
#define MAG_DECLINATION 0//-1327440	// magnetic declination (radians * 2^24).  In Indianapolis, this is -1327440, corresponding to -4.5333333 degrees

//Do not add code below this line
#endif /* IMU_Kalman_H_ */
