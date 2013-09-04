/*
 * GlobalVariables.cpp
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */
#include "GlobalVariables.h"

unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds
unsigned long loopdeltatime_us;		// time between last loop and this loop, in microseconds
boolean newmagdata = 0;				// this flag tells new magnetic data was captured on this loop
boolean newmpudata = 0;				// this flag tells new MPU data was captured on this loop

_lAccum rollrate_meas = 0;			// aircraft roll rate about x-axis, in radians/sec.  a positive value indicates increasing roll towards right.  measured from IMU
_lAccum pitchrate_meas = 0;			// aircraft pitch rate about y-axis, in radians/sec.  a positive value indicates increasing pitch upwards.  measured from IMU
_lAccum yawrate_meas = 0;			// aircraft yaw rate about z-axis, in radians/sec.  a positive value indicates increasing yaw right.  measured from IMU
_lAccum accelX_meas = 0;			// measured acceleration in x-direction, m/s^2.  measured from IMU
_lAccum accelY_meas = 0;			// measured acceleration in y-direction, m/s^2.  measured from IMU
_lAccum accelZ_meas = 0;			// measured acceleration in z-direction, m/s^2.  measured from IMU
_lAccum roll_fromAccel = 0;			// calculated roll angle (rad), using raw data from accels to determine where gravity vector is.
_lAccum pitch_fromAccel = 0;		// calculated roll angle (rad), using raw data from accels to determine where gravity vector is.
_lAccum	heading_rad = 0;			// heading, in radians (*2^24).  Measurement from magnetometer.
_lAccum roll = 0;					// aircraft roll about x-axis, in radians.  a positive value indicates roll right.  estimate from Kalman filter
_lAccum pitch = 0;					// aircaft pitch about y-axis, in radians.  a positive value indicates pitch up.  estimate from Kalman filter
_lAccum yaw = 0;					// aircraft yaw about z-axis, in radians.  These should be aligned with compass coordinates. (north = zero radians).  estimate from Kalman filter.
_lAccum rollrate = 0;				// aircraft roll rate about x-axis, in radians/sec.  a positive value indicates roll right.  estimate from Kalman filter
_lAccum pitchrate = 0;				// aircaft pitch rate about y-axis, in radians/sec.  a positive value indicates pitch up.  estimate from Kalman filter
_lAccum yawrate = 0;				// aircraft yaw rate about z-axis, in radians/sec.  These should be aligned with compass coordinates. (north = zero radians).  estimate from Kalman filter.

int		heading_deg = 0;			// heading, in degrees
_lAccum	magX_Ga = 0;				// X-axis magnetic field, in Gauss (*2^24)
_lAccum	magY_Ga = 0;				// Y-axis magnetic field, in Gauss (*2^24)
_lAccum	magZ_Ga = 0;				// Z-axis magnetic field, in Gauss (*2^24)
byte mag_datacount = 0;				// number of data ready in magnetometer
_lAccum templAccum1 = 0;			// temporary _lAccum for debugging purposes
_lAccum templAccum2 = 0;			// temporary _lAccum for debugging purposes
_lAccum templAccum3 = 0;			// temporary _lAccum for debugging purposes
_lAccum templAccum4 = 0;			// temporary _lAccum for debugging purposes
