/*
 * GyroKalman.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Rancuret
 */

#ifndef GYROKALMAN_H_
#define GYROKALMAN_H_

// GyroKalman Class Definition
/*
 * Class:		GyroKalman
 * Function:	NA
 * Scope:		global
 * Arguments:	NA
 * Description:	This class contains the GyroKalman functions and data.
 * 				These functions provide Kalman filter functionality for
 * 				one of the three gyroscope measurements.
 */
class GyroKalman {
private:


public:
	GyroKalman(void);				// Constructor for GyroKalman class

};	// end of class GyroKalman

// global classes
extern GyroKalman XAxisGyroKalman;	// Kalman filter for x-axis gyro measurement
extern GyroKalman YAxisGyroKalman;	// Kalman filter for y-axis gyro measurement
extern GyroKalman ZAxisGyroKalman;	// Kalman filter for z-axis gyro measurement

#endif /* GYROKALMAN_H_ */
