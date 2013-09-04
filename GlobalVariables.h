/*
 * GlobalVariables.h
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "IMU_Kalman.h"

extern unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds
extern unsigned long loopdeltatime_us;		// time between last loop and this loop, in microseconds
extern boolean newmagdata;					// this flag tells new magnetic data was captured on this loop
extern boolean newmpudata;					// this flag tells new MPU data was captured on this loop

extern _lAccum rollrate_meas;				// aircraft roll rate about x-axis, in radians/sec.  a positive value indicates increasing roll towards right.  measured from IMU
extern _lAccum pitchrate_meas;				// aircraft pitch rate about y-axis, in radians/sec.  a positive value indicates increasing pitch upwards.  measured from IMU
extern _lAccum yawrate_meas;				// aircraft yaw rate about z-axis, in radians/sec.  a positive value indicates increasing yaw right.  measured from IMU
extern _lAccum accelX_meas;					// measured acceleration in x-direction, m/s^2.  measured from IMU
extern _lAccum accelY_meas;					// measured acceleration in y-direction, m/s^2.  measured from IMU
extern _lAccum accelZ_meas;					// measured acceleration in z-direction, m/s^2.  measured from IMU
extern _lAccum roll_fromAccel;				// calculated roll angle (rad), using raw data from accels to determine where gravity vector is.
extern _lAccum pitch_fromAccel;				// calculated roll angle (rad), using raw data from accels to determine where gravity vector is.
extern _lAccum heading_rad;					// heading, in radians (*2^24).  Measurement from magnetometer.
extern _lAccum roll;						// aircraft roll about x-axis, in radians.  a positive value indicates roll right
extern _lAccum pitch;						// aircaft pitch about y-axis, in radians.  a positive value indicates pitch up
extern _lAccum yaw;							// aircraft yaw about z-axis, in radians.  These should be aligned with compass coordinates. (north = zero radians)
extern _lAccum rollrate;					// aircraft roll rate about x-axis, in radians/sec.  a positive value indicates roll right.  estimate from Kalman filter
extern _lAccum pitchrate;					// aircaft pitch rate about y-axis, in radians/sec.  a positive value indicates pitch up.  estimate from Kalman filter
extern _lAccum yawrate;						// aircraft yaw rate about z-axis, in radians/sec.  These should be aligned with compass coordinates. (north = zero radians).  estimate from Kalman filter.

extern int		heading_deg;				// heading, in degrees
extern _lAccum	magX_Ga;					// X-axis magnetic field, in Gauss (*2^24)
extern _lAccum	magY_Ga;					// Y-axis magnetic field, in Gauss (*2^24)
extern _lAccum	magZ_Ga;					// Z-axis magnetic field, in Gauss (*2^24)
extern byte mag_datacount;					// number of data ready in magnetometer
extern _lAccum templAccum1;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum2;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum3;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum4;					// temporary _lAccum for debugging purposes
#endif /* GLOBALVARIABLES_H_ */
