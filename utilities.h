/*
 * utilities.h
 *
 *  Created on: Sep 2, 2013
 *      Author: Rancuret
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "IMU_Kalman.h"

// global functions
extern void CheckForData(void);					// Checks for and reads data from MPU and magnetometer
extern void Calculate_Kalman_Estimates(void);	// calculates all kalman state estimates, based on whatever new measurements are ready
extern void Debug_Mag_Messages(void);			// this function will cause some basic debugging from the magnetometer
extern void Print_Filter_Debug_Out(void);		// this function will cause filter outputs to be printed over serial line every 10th time it is called



#endif /* UTILITIES_H_ */
