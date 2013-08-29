/*
 * GyroKalman.cpp
 *
 *  Created on: Aug 25, 2013
 *      Author: Rancuret
 *
 *      Description:	provides Gyroscope measurement Kalman Filter for ArduIMU V3
 */

#include "GyroKalman.h"

// definitions
GyroKalman XAxisGyroKalman;	// Kalman filter for x-axis gyro measurement
GyroKalman YAxisGyroKalman;	// Kalman filter for y-axis gyro measurement
GyroKalman ZAxisGyroKalman;	// Kalman filter for z-axis gyro measurement

// GyroKalman public functions

/*
 * Class:		GyroKalman
 * Function:	GyroKalman()
 * Scope:		Public
 * Arguments:	None
 * Description:	Constructor for GyroKalman object
 */
GyroKalman::GyroKalman(void) {

	return;
} // end of GyroKalman()
