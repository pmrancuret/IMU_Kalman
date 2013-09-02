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
	rate_proc_noise_var = 0;				// Initial value of 0.  variance in process noise affecting angular rate (rad^2/s^2*2^24).  In the case of an aircraft, this could be noise such as wind.
	rate_obs_noise_var = 0;					// Initial value of 0.  variance in observation noise affecting rate measurement (rad^2/s^2*2^24).  This is the variance of the gyroscope signal noise.
	angle_proc_noise_var = 0;				// Initial value of 0.  variance in process noise affecting angle (rad^2 * 2^24).
	angle_obs_noise_var = 0;				// Initial value of 0.  variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise.
	angle_pred = 0;							// Initial value of 0.  predicted angle (radians*2^24) at current time, based only on past information
	angle_est = 0;							// Initial value of 0.  estimated angle (radians*2^24) at current time, updated with latest measurements
	rate_pred = 0;							// Initial value of 0.  predicted angular velocity (radians/sec * 2^24) at current time, based only on past information
	rate_est = 0;							// Initial value of 0.  estimated angular velocity (radians/sec * 2^24), updated with latest measurements
	est_cov_apriori[0] = 0;					// Initial value of 0.  a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[1] = 0;					// Initial value of 0.  a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[2] = 0;					// Initial value of 0.  a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[3] = 0;					// Initial value of 0.  a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_aposteriori[0] = 0;				// Initial value of 0.  a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[1] = 0;				// Initial value of 0.  a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[2] = 0;				// Initial value of 0.  a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[3] = 0;				// Initial value of 0.  a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)

	return;
} // end of GyroKalman()

/*
 * Class:		GyroKalman
 * Function:	Predict()
 * Scope:		Public
 * Arguments:	long	StepTime_us		- step time taken between this prediction and last estimation step, in microseconds
 * Description:	performs prediction step based on past information, only knowing time step to take
 */
void GyroKalman::Predict(long StepTime_us){
	Predict(StepTime_us,0);		// calls prediction step with zero known control input
	return;
} // end of Predict()

/*
 * Class:		GyroKalman
 * Function:	Predict()
 * Scope:		Public
 * Arguments:	long	StepTime_us				- step time taken between this prediction and last estimation step, in microseconds.  Limit of 999992 (0.999992 seconds)
 * 				_lAccum	rateActuation_radps2	- known controller's angular acceleration (from torque) input to system at last time interval, in rad/sec^2
 * Description:	performs prediction step based on past information, only knowing time step to take.  Includes optional
 * 				controller actuation input (rad/s*2^24), if known.
 */
void GyroKalman::Predict(long StepTime_us, _lAccum rateActuation_radps2){
	_lAccum	StepTime_slk;			// step time in seconds*2^24 (using _lAccum datatype, instead of long microseconds)
	_lAccum StepTime_x_input;		// step time times rate actuation (rad/sec * 2^24)
	_lAccum StepTime_x_lastcov;		// step time times  last element of a-posteriori estimate covariance matrix

	StepTime_slk = (_lAccum)((constrain(StepTime_us,1,999992)*MICROSECONDTIMES2_32PWR)>>8);	// rewrite step time in seconds * 2^24, which is the _lAccum datatype.  This operation constrains to a limit of 0.999992 seconds.
	if (rateActuation_radps2 == 0)								// if no control input was specified
	{
		angle_pred = angle_est + lmullk(rate_est,StepTime_slk);	// calculate predicted angle
		rate_pred = rate_est;									// calculate angular velocity prediction
	}
	else														// if control input was specified
	{
		StepTime_x_input = lmullk(rateActuation_radps2,StepTime_slk);				// calculate step time times rate actuation (rad/sec * 2^24)
		angle_pred = angle_est + lmullk(rate_est,StepTime_slk) + lmullk(StepTime_x_input,StepTime_slk);	// calculate predicted angle
		rate_pred = rate_est + StepTime_x_input;			// calculate angular velocity prediction
	}

	StepTime_x_lastcov = lmullk(est_cov_aposteriori[3],StepTime_slk);								// calculate step time times  last element of a-posteriori estimate covariance matrix
	est_cov_apriori[0] = 	est_cov_aposteriori[0] + lmullk(est_cov_aposteriori[1],StepTime_slk) +
							lmullk(est_cov_aposteriori[2],StepTime_slk) +
							lmullk(StepTime_slk,StepTime_x_lastcov) +
							angle_proc_noise_var;													// update first row, first column of a-priori estimate covariance matrix
	est_cov_apriori[1] = 	est_cov_aposteriori[1] + StepTime_x_lastcov;							// update first row, second column of a-priori estimate covariance matrix
	est_cov_apriori[2] =	est_cov_aposteriori[2] + StepTime_x_lastcov;							// update second row, first column of a-priori estimate covariance matrix
	est_cov_apriori[3] =	est_cov_aposteriori[3] + rate_proc_noise_var;							// update second row, second column of a-priori estimate covariance matrix

	return;
} // end of Predict()

/*
 * Class:		GyroKalman
 * Function:	Update_with_NoData()
 * Scope:		Public
 * Arguments:	None
 * Description:	performs update step without any measured data - really just skips the update step and places prediction
 * 				in estimate instead
 */
void GyroKalman::Update_with_NoData(void){
	angle_est = angle_pred;			// set angle estimate to predicted value (no measurement)
	rate_est = rate_pred;			// set rate estimate to predicted value (no measurement)
	return;
} // end of Update_with_NoData()

/*
 * Class:		GyroKalman
 * Function:	Update_with_Angle()
 * Scope:		Public
 * Arguments:	_lAccum	angle	- angle measured at this time step, in radians *2^24
 * Description:	performs update step with measured angle data (from compass), but no rate data.
 */
void GyroKalman::Update_with_Angle(_lAccum angle){
	_lAccum meas_residual;				// measurement residual
	_lAccum residual_var;				// residual variance
	_lAccum residual_var_inv;			// inverse of residual variance;
	_lAccum opt_gain[2];				// optimal Kalman gain vector

	// calculate residual and variances
	meas_residual = angle - angle_pred;	// calculate measurement residual
	residual_var = est_cov_apriori[0] + angle_obs_noise_var;	// calculate residual variance
	residual_var_inv = ldivlk(ONE_LK,residual_var);				// calculate inverse of residual variance

	// Calculate the optimal kalman gain
	opt_gain[0] = lmullk(residual_var_inv,est_cov_apriori[0]);	// calculate first element of optimal Kalman gain
	opt_gain[1] = lmullk(residual_var_inv,est_cov_apriori[2]);	// calculate second element of optimal Kalman gain

	// Update state estimates
	angle_est = angle_pred + lmullk(opt_gain[0],meas_residual);	// update (a posteriori) angle state estimate
	rate_est = rate_pred + lmullk(opt_gain[1],meas_residual);	// update (a posteriori) rate state estimate

	// Update Estimate Covariance Matrix
	est_cov_aposteriori[0] = est_cov_apriori[0] - lmullk(est_cov_apriori[0],opt_gain[0]);	// update first row, first column of estimate covariance matrix
	est_cov_aposteriori[1] = est_cov_apriori[1] - lmullk(est_cov_apriori[1],opt_gain[0]);	// update first row, second column of estimate covariance matrix
	est_cov_aposteriori[2] = est_cov_apriori[2] - lmullk(est_cov_apriori[0],opt_gain[1]);	// update second row, first column of estimate covariance matrix
	est_cov_aposteriori[3] = est_cov_apriori[3] - lmullk(est_cov_apriori[1],opt_gain[1]);	// update second row, second column of estimate covariance matrix

	return;
} // end of Update_with_Angle()

