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
	_lAccum angle_pred;				// predicted angle (radians*2^24) at current time, based only on past information
	_lAccum angle_est;				// estimated angle (radians*2^24) at current time, updated with latest measurements
	_lAccum angle_est_last;			// previous estimated angle (radians*2^24)
	_lAccum angular_vel_pred;		// predicted angular velocity (radians/sec * 2^24) at current time, based only on past information
	_lAccum angular_vel_est;		// estimated angular velocity (radians/sec * 2^24), updated with latest measurements
	_lAccum angular_vel_est_last;	// previous estimated angular velocity (radians/sec * 2^24)


public:
	GyroKalman(void);				// Constructor for GyroKalman class

};	// end of class GyroKalman

// global classes
extern GyroKalman XAxisGyroKalman;	// Kalman filter for x-axis gyro measurement
extern GyroKalman YAxisGyroKalman;	// Kalman filter for y-axis gyro measurement
extern GyroKalman ZAxisGyroKalman;	// Kalman filter for z-axis gyro measurement

#endif /* GYROKALMAN_H_ */
