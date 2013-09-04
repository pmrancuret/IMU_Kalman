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
#include "utilities.h"

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
//#define DEBUG_MAGNETOMETER		// defining this variable will cause some basic debugging from the magnetometer
#define DEBUG_PRINTFILTEROUTS		// defining this variable will cause filter outputs to be printed over serial line every 10th time it is called

//add your function definitions for the project IMU_Kalman here

#define SERIAL_DATARATE 38400	// serial line datarate, baud

#define FASTLOOP_TIME_US 7500	// fast loop sample rate, in microseconds.

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

// variances for each Kalman filter
#define XGYRO_ANGLEPROCVAR	itolk(1)>>2		// X-axis gyro angle process variance of 1/4 rad
#define YGYRO_ANGLEPROCVAR	itolk(1)>>2		// Y-axis gyro angle process variance of 1/4 rad
#define ZGYRO_ANGLEPROCVAR	itolk(1)>>2		// Z-axis gyro angle process variance of 1/4 rad
#define XGYRO_ANGLEOBSVAR	itolk(1)>>2		// X-axis gyro angle observation variance of 1/4 rad
#define YGYRO_ANGLEOBSVAR	itolk(1)>>2		// Y-axis gyro angle observation variance of 1/4 rad
#define ZGYRO_ANGLEOBSVAR	itolk(1)>>2		// Z-axis gyro angle observation variance of 1/4 rad
#define XGYRO_RATEPROCVAR	itolk(1)>>2		// X-axis gyro rate process variance of 1/4 rad/sec
#define YGYRO_RATEPROCVAR	itolk(1)>>2		// Y-axis gyro rate process variance of 1/4 rad/sec
#define ZGYRO_RATEPROCVAR	itolk(1)>>2		// Z-axis gyro rate process variance of 1/4 rad/sec
#define XGYRO_RATEOBSVAR	itolk(1)>>2		// X-axis gyro rate observation variance of 1/4 rad/sec
#define YGYRO_RATEOBSVAR	itolk(1)>>2		// Y-axis gyro rate observation variance of 1/4 rad/sec
#define ZGYRO_RATEOBSVAR	itolk(1)>>2		// Z-axis gyro rate observation variance of 1/4 rad/sec
#define XGYRO_INITIALANGLE	0				// X-axis gyro initial angle, rad
#define YGYRO_INITIALANGLE	0				// Y-axis gyro initial angle, rad
#define ZGYRO_INITIALANGLE	0				// Z-axis gyro initial angle, rad
#define XGYRO_INITIALRATE	0				// X-axis gyro initial rate, rad/sec
#define YGYRO_INITIALRATE	0				// Y-axis gyro initial rate, rad/sec
#define ZGYRO_INITIALRATE	0				// Z-axis gyro initial rate, rad/sec
#define XGYRO_ANGLEESTVAR	itolk(1)>>2		// X-axis initial angle estimation variance, rad^2
#define YGYRO_ANGLEESTVAR	itolk(1)>>2		// Y-axis initial angle estimation variance, rad^2
#define ZGYRO_ANGLEESTVAR	itolk(1)>>2		// Z-axis initial angle estimation variance, rad^2
#define XGYRO_RATEESTVAR	itolk(1)>>2		// X-axis initial rate estimation variance, rad^2/sec^2
#define YGYRO_RATEESTVAR	itolk(1)>>2		// Y-axis initial rate estimation variance, rad^2/sec^2
#define ZGYRO_RATEESTVAR	itolk(1)>>2		// Z-axis initial rate estimation variance, rad^2/sec^2

//Do not add code below this line
#endif /* IMU_Kalman_H_ */
