/*
 * utilities.h
 *
 *  Created on: Sep 2, 2013
 *      Author: Rancuret
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "IMU_Kalman.h"

#define GRAVITYMPS2_LK	164528285	// gravity's force in m/s^2 (as an _lAccum number, meaning *2^24)
#define FIVEDEGREES_LK	1464088		// 5 degrees, stored as an _lAccum number that really means radians * 2^24

#ifndef NULL
#define NULL ((void*)0)
#endif

#define ToDeg100(x) 	((int)((((long)x)>>19) * 179))
#define ToDeg50(x)		((int)((((long)x)>>20) * 179))


/*
 * Class:		CalcMeanAndVariance
 * Function:	NA
 * Scope:		global
 * Arguments:	None
 * Description:	used to calculate the mean and variance of a signal online
 */
class CalcMeanAndVariance{
private:
	long n;							// number of samples taken so far
	_lAccum mean;					// mean calculated so far
	_lAccum variance;				// variance calculated so far
	_lAccum M2;						// M2_k2 = M2_k1 + (x-mean_k2)(x-mean_k1)

public:
	CalcMeanAndVariance(void);		// constructor for this class
	void Initialize(_lAccum mean_init, _lAccum var_init);				// initializes the mean and variance values
	_lAccum CalcNextValue(_lAccum measured, _lAccum * variance);	// calculates the next value for mean and variance

};

// global functions
extern void PrintBinaryData(void);				// prints binary data on Serial port to be used by autopilot
extern void Initialize_System(void);			// initializes the IMU system
extern void CheckForData(void);					// Checks for and reads data from MPU and magnetometer
extern void Calculate_Kalman_Estimates(void);	// calculates all kalman state estimates, based on whatever new measurements are ready
extern void Debug_Mag_Messages(void);			// this function will cause some basic debugging from the magnetometer
extern void Print_Filter_Debug_Out(void);		// this function will cause filter outputs to be printed over serial line every 10th time it is called



#endif /* UTILITIES_H_ */
