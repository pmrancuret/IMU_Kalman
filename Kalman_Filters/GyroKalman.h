/*
 * GyroKalman.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Rancuret
 */

#ifndef GYROKALMAN_H_
#define GYROKALMAN_H_

#include "avrfix.h"
#include "Arduino.h"

// definitions
#define	MICROSECONDTIMES2_32PWR		4295	// one microsecond (1e-6 sec), rescaled in seconds times 2^32.
#define ONE_LK						0x1000000	// one times 2^32
#define MAX_LK						0x7FFFFFFF	// maximum value of _lAccum data type
#ifndef PILK
#define PILK						52707178	// This value represents pi in the _lAccum data type (*2^24)
#endif
#ifndef TWOPILK
#define TWOPILK					105414357	// This value represents 2*pi in the _lAccum data type (*2^24)
#endif

// GyroKalman Class Definition
/*
 * Class:		GyroKalman
 * Function:	NA
 * Scope:		global
 * Arguments:	NA
 * Description:	This class contains the GyroKalman functions and data.
 * 				These functions provide Kalman filter functionality for
 * 				one of the three gyroscope measurements.  Assumes covariance
 * 				matrixes for process and observation noise inputs are diagonal.
 */
class GyroKalman {
private:
	// diagonal elements of process noise covariance matrix
	_lAccum angle_proc_noise_var;	// variance in process noise affecting angle (rad^2 * 2^24).
	_lAccum rate_proc_noise_var;	// variance in process noise affecting angular rate (rad^2/s^2*2^24).  In the case of an aircraft, this could be noise such as wind, or unknown control inputs.

	// diagonal elements of observation noise covariance matrix
	_lAccum angle_obs_noise_var;	// variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise.
	_lAccum rate_obs_noise_var;		// variance in observation noise affecting rate measurement (rad^2/s^2*2^24).  This is the variance of the gyroscope signal noise.

	// Estimate covariance matrix.  four-element array where first two elements are first row, and second two elements are second row
	_lAccum est_cov_apriori[4];		// a-priori estimate covariance matrix (calculated after prediction step, before updating with measurements)
	_lAccum est_cov_aposteriori[4];	// a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)

	// state predictions.  These predictions are calculated based on time step taken, without accounting for measurements yet (only uses past data).
	_lAccum angle_pred;				// predicted angle (radians*2^24) at current time, based only on past information
	_lAccum rate_pred;				// predicted angular velocity (radians/sec * 2^24) at current time, based only on past information

	// state estimates.  These estimates have been updated with new measurement data
	_lAccum angle_est;				// estimated angle (radians*2^24) at current time, updated with latest measurements
	_lAccum rate_est;				// estimated angular velocity (radians/sec * 2^24), updated with latest measurements

	void Predict(long StepTime_us);									// performs prediction step based on past information, only knowing time step to take
	void Predict(long StepTime_us, _lAccum rateActuation_radps2);	// performs prediction step based on past information, only knowing time step to take.  Includes optional controller actuation input (rad/s*2^24), if known.
	void Update_with_NoData(void);									// performs update step without any measured data - really just skips the update step and places prediction in estimate instead
	void Update_with_Angle(_lAccum angle);							// performs update step with measured angle data (from compass), but no rate data.
	void Update_with_Rate(_lAccum rate);							// performs update step with measured rate data (from gyro), but no angle data.
	void Update_with_Angle_and_Rate(_lAccum angle, _lAccum rate);	// performs update step with both measured angle and rate data

public:
	GyroKalman(void);				// Constructor for GyroKalman class
	void SetAngleObsVar(_lAccum var);								// sets variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise. minimum value of one rad^2, max of 128 rad^2
	void Initialize(	_lAccum angleprocnoisevar,
						_lAccum rateprocnoisevar,
						_lAccum angleobsnoisevar,
						_lAccum rateobsnoisevar,
						_lAccum InitialAngle,
						_lAccum InitialRate,
						_lAccum InitialAngleEstVar,
						_lAccum InitialRateEstVar);					// Initializes the Kalman Filter with specified variances, state estimates, and estimate variances
	void SetAngleProcNoiseVar(_lAccum angleprocnoisevar);			// Sets Variance of process noise affecting angle, rad^2 *2^24.
	void SetRateProcNoiseVar(_lAccum rateprocnoisevar);				// Sets Variance of process noise affecting rate, rad^2/sec^2 *2^24.
	void SetAngleObsNoiseVar(_lAccum angleobsnoisevar);				// Sets Variance of observation noise affecting angle, rad^2 *2^24.
	void SetRateObsNoiseVar(_lAccum rateobsnoisevar);				// Sets Variance of observation noise affecting rate, rad^2/sec^2 *2^24.
	void Est_NoCtrl_NoMeas(long StepTime_us);						// Performs estimation step with no known control inputs and no measured states
	void Est_NoCtrl_MeasAngle(long StepTime_us,_lAccum angle);		// Performs estimation step with no known control inputs and measured angle (but no measured rate)
	void Est_NoCtrl_MeasRate(long StepTime_us,_lAccum rate);		// Performs estimation step with no known control inputs and measured rate (but no measured angle)
	void Est_NoCtrl_MeasAngleAndRate(long StepTime_us,
						_lAccum angle,_lAccum rate);				// Performs estimation step with no known control inputs, but with measured angle and rate
	void Est_Ctrl_NoMeas(long StepTime_us,
						_lAccum rateActuation_radps2);				// Performs estimation step with known control inputs and no measured states
	void Est_Ctrl_MeasAngle(long StepTime_us,
						_lAccum rateActuation_radps2,_lAccum angle);// Performs estimation step with known control inputs and measured angle (but no measured rate)
	void Est_Ctrl_MeasRate(long StepTime_us,
						_lAccum rateActuation_radps2,_lAccum rate);	// Performs estimation step with known control inputs and measured rate (but no measured angle)
	void Est_Ctrl_MeasAngleAndRate(long StepTime_us,
						_lAccum rateActuation_radps2,_lAccum angle,
						_lAccum rate);								// Performs estimation step with known control inputs, but with measured angle and rate
	_lAccum GetAngle(void);											// returns estimated angle, in radians
	_lAccum GetRate(void);											// returns estimated rate, in radians/sec
	_lAccum GetEstCov(byte i);										// returns the desired element of the estimate covariance matrix.

};	// end of class GyroKalman

// global classes
extern GyroKalman XAxisGyroKalman;	// Kalman filter for x-axis gyro measurement
extern GyroKalman YAxisGyroKalman;	// Kalman filter for y-axis gyro measurement
extern GyroKalman ZAxisGyroKalman;	// Kalman filter for z-axis gyro measurement

#endif /* GYROKALMAN_H_ */
