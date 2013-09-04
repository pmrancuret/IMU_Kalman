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
 * Function:	Initialize()
 * Scope:		Public
 * Arguments:	_lAccum	angleprocnoisevar	- Variance of process noise affecting angle, rad^2 *2^24.
 * 				_lAccum rateprocnoisevar	- Variance of process noise affecting rate, rad^2/sec^2 *2^24.
 * 				_lAccum angleobsnoisevar	- Variance of observation noise affecting angle, rad^2 *2^24.
 * 				_lAccum rateobsnoisevar		- Variance of observation noise affecting rate, rad^2/sec^2 *2^24.
 * 				_lAccum InitialAngle		- Initial angle estimate, rad *2^24
 * 				_lAccum InitialRate			- Initial rate estimate, rad/sec *2^24
 * 				_lAccum InitialAngleEstVar	- Initial angle estimate variance, rad^2 *2^24.
 * 				_lAccum InitialRateEstVar	- Initial rate estimate variance, rad^2/sec^2 *2^24.
 * Description:	Initializes the Kalman Filter with specified variances, state estimates, and estimate variances
 */
void GyroKalman::Initialize(	_lAccum angleprocnoisevar,
								_lAccum rateprocnoisevar,
								_lAccum angleobsnoisevar,
								_lAccum rateobsnoisevar,
								_lAccum InitialAngle,
								_lAccum InitialRate,
								_lAccum InitialAngleEstVar,
								_lAccum InitialRateEstVar){
	rate_proc_noise_var = rateprocnoisevar;			// variance in process noise affecting angular rate (rad^2/s^2*2^24).  In the case of an aircraft, this could be noise such as wind.
	rate_obs_noise_var = rateobsnoisevar;			// variance in observation noise affecting rate measurement (rad^2/s^2*2^24).  This is the variance of the gyroscope signal noise.
	angle_proc_noise_var = angleprocnoisevar;		// variance in process noise affecting angle (rad^2 * 2^24).
	angle_obs_noise_var = angleobsnoisevar;			// variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise.
	angle_pred = InitialAngle;						// predicted angle (radians*2^24) at current time, based only on past information
	angle_est = InitialAngle;						// estimated angle (radians*2^24) at current time, updated with latest measurements
	rate_pred = InitialRate;						// predicted angular velocity (radians/sec * 2^24) at current time, based only on past information
	rate_est = InitialRate;							// estimated angular velocity (radians/sec * 2^24), updated with latest measurements
	est_cov_apriori[0] = InitialAngleEstVar;		// a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[1] = 0;							// a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[2] = 0;							// a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_apriori[3] = InitialRateEstVar;			// a-priori estimate covariance matrix (calculated after prediction step, befure updating with measurements)
	est_cov_aposteriori[0] = InitialAngleEstVar;	// a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[1] = 0;						// a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[2] = 0;						// a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	est_cov_aposteriori[3] = InitialRateEstVar;		// a-posteriori estimate covariance matrix (calculated after estimates are updated with measurements)
	return;
} // end of Initialize()

/*
 * Class:		GyroKalman
 * Function:	SetAngleProcNoiseVar()
 * Scope:		Public
 * Arguments:	_lAccum	angleprocnoisevar	- Variance of process noise affecting angle, rad^2 *2^24.
 * Description:	Sets Variance of process noise affecting angle, rad^2 *2^24.
 */
void GyroKalman::SetAngleProcNoiseVar(_lAccum angleprocnoisevar){
	angle_proc_noise_var = angleprocnoisevar;		// variance in process noise affecting angle (rad^2 * 2^24).
	return;
} // end of SetAngleProcNoiseVar()

/*
 * Class:		GyroKalman
 * Function:	SetRateProcNoiseVar()
 * Scope:		Public
 * Arguments:	_lAccum rateprocnoisevar	- Variance of process noise affecting rate, rad^2/sec^2 *2^24.
 * Description:	Sets Variance of process noise affecting rate, rad^2/sec^2 *2^24.
 */
void GyroKalman::SetRateProcNoiseVar(_lAccum rateprocnoisevar){
	rate_proc_noise_var = rateprocnoisevar;			// variance in process noise affecting angular rate (rad^2/s^2*2^24).  In the case of an aircraft, this could be noise such as wind.
	return;
} // end of SetRateProcNoiseVar()

/*
 * Class:		GyroKalman
 * Function:	SetAngleObsNoiseVar()
 * Scope:		Public
 * Arguments:	_lAccum angleobsnoisevar	- Variance of observation noise affecting angle, rad^2 *2^24.
 * Description:	Sets variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise.
 */
void GyroKalman::SetAngleObsNoiseVar(_lAccum angleobsnoisevar){
	angle_obs_noise_var = angleobsnoisevar;			// variance in observation noise affecting angle (rad^2 * 2^24).  This is the variance of the compass signal noise.
	return;
} // end of SetAngleObsNoiseVar()

/*
 * Class:		GyroKalman
 * Function:	SetRateObsNoiseVar()
 * Scope:		Public
 * Arguments:	_lAccum rateobsnoisevar		- Variance of observation noise affecting rate, rad^2/sec^2 *2^24.
 * Description:	Sets variance in observation noise affecting rate measurement (rad^2/s^2*2^24).  This is the variance of the gyroscope signal noise.
 */
void GyroKalman::SetRateObsNoiseVar(_lAccum rateobsnoisevar){
	rate_obs_noise_var = rateobsnoisevar;			// variance in observation noise affecting rate measurement (rad^2/s^2*2^24).  This is the variance of the gyroscope signal noise.
	return;
} // end of SetRateObsNoiseVar()

/*
 * Class:		GyroKalman
 * Function:	Est_NoCtrl_NoMeas()
 * Scope:		Public
 * Arguments:	long	StepTime_us		- step time taken between this prediction and last estimation step, in microseconds
 * Description:	Performs estimation step with no known control inputs and no measured states
 */
void GyroKalman::Est_NoCtrl_NoMeas(long StepTime_us){
	Predict(StepTime_us);						// predict state estimates and estimate covariances based on past data alone
	Update_with_NoData();						// update state estimates and covariances with new measurements (none in this case)
	return;
} // end of Est_NoCtrl_NoMeas()

/*
 * Class:		GyroKalman
 * Function:	Est_NoCtrl_MeasAngle()
 * Scope:		Public
 * Arguments:	long	StepTime_us		- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	angle			- angle measured at this time step, in radians *2^24
 * Description:	Performs estimation step with no known control inputs and measured angle (but no measured rate)
 */
void GyroKalman::Est_NoCtrl_MeasAngle(long StepTime_us,_lAccum angle){
	Predict(StepTime_us);						// predict state estimates and estimate covariances based on past data alone
	Update_with_Angle(angle);					// update state estimates and covariances with new measurements (angle measurement in this case)
	return;
} // end of Est_NoCtrl_MeasAngle()

/*
 * Class:		GyroKalman
 * Function:	Est_NoCtrl_MeasRate()
 * Scope:		Public
 * Arguments:	long	StepTime_us		- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	rate			- rate measured at this time step, in radians/sec *2^24
 * Description:	Performs estimation step with no known control inputs and measured rate (but no measured angle)
 */
void GyroKalman::Est_NoCtrl_MeasRate(long StepTime_us,_lAccum rate){
	Predict(StepTime_us);						// predict state estimates and estimate covariances based on past data alone
	Update_with_Rate(rate);					// update state estimates and covariances with new measurements (rate measurement in this case)
	return;
} // end of Est_NoCtrl_MeasRate()

/*
 * Class:		GyroKalman
 * Function:	Est_NoCtrl_MeasAngleAndRate()
 * Scope:		Public
 * Arguments:	long	StepTime_us		- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	angle			- angle measured at this time step, in radians *2^24
 * 				_lAccum	rate			- rate measured at this time step, in radians/sec *2^24
 * Description:	Performs estimation step with no known control inputs, but with measured angle and rate
 */
void GyroKalman::Est_NoCtrl_MeasAngleAndRate(long StepTime_us,
					_lAccum angle,_lAccum rate){
	Predict(StepTime_us);						// predict state estimates and estimate covariances based on past data alone
	Update_with_Angle_and_Rate(angle,rate);		// update state estimates and covariances with new measurements (both angle and rate measurements in this case)
	return;
} // end of Est_NoCtrl_MeasAngleAndRate()

/*
 * Class:		GyroKalman
 * Function:	Est_Ctrl_NoMeas()
 * Scope:		Public
 * Arguments:	long	StepTime_us				- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	rateActuation_radps2	- known controller's angular acceleration (from torque) input to system at last time interval, in rad/sec^2
 * Description:	Performs estimation step with known control inputs and no measured states
 */
void GyroKalman::Est_Ctrl_NoMeas(long StepTime_us,
					_lAccum rateActuation_radps2){
	Predict(StepTime_us,rateActuation_radps2);	// predict state estimates and estimate covariances based on past data, with known control inputs
	Update_with_NoData();						// update state estimates and covariances with new measurements (none in this case)
	return;
} // end of Est_Ctrl_NoMeas()

/*
 * Class:		GyroKalman
 * Function:	Est_Ctrl_MeasAngle()
 * Scope:		Public
 * Arguments:	long	StepTime_us				- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	rateActuation_radps2	- known controller's angular acceleration (from torque) input to system at last time interval, in rad/sec^2
 * 				_lAccum	angle					- angle measured at this time step, in radians *2^24
 * Description:	Performs estimation step with known control inputs and measured angle (but no measured rate)
 */
void GyroKalman::Est_Ctrl_MeasAngle(long StepTime_us,
					_lAccum rateActuation_radps2,_lAccum angle){
	Predict(StepTime_us,rateActuation_radps2);	// predict state estimates and estimate covariances based on past data, with known control inputs
	Update_with_Angle(angle);					// update state estimates and covariances with new measurements (angle measurement in this case)
	return;
} // end of Est_Ctrl_MeasAngle()

/*
 * Class:		GyroKalman
 * Function:	Est_Ctrl_MeasRate()
 * Scope:		Public
 * Arguments:	long	StepTime_us				- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	rateActuation_radps2	- known controller's angular acceleration (from torque) input to system at last time interval, in rad/sec^2
 * 				_lAccum	rate					- rate measured at this time step, in radians/sec *2^24
 * Description:	Performs estimation step with known control inputs and measured rate (but no measured angle)
 */
void GyroKalman::Est_Ctrl_MeasRate(long StepTime_us,
					_lAccum rateActuation_radps2,_lAccum rate){
	Predict(StepTime_us,rateActuation_radps2);	// predict state estimates and estimate covariances based on past data, with known control inputs
	Update_with_Rate(rate);					// update state estimates and covariances with new measurements (rate measurement in this case)
	return;
} // end of Est_Ctrl_MeasRate

/*
 * Class:		GyroKalman
 * Function:	Est_Ctrl_MeasAngleAndRate()
 * Scope:		Public
 * Arguments:	long	StepTime_us				- step time taken between this prediction and last estimation step, in microseconds
 * 				_lAccum	rateActuation_radps2	- known controller's angular acceleration (from torque) input to system at last time interval, in rad/sec^2
 * 				_lAccum	angle					- angle measured at this time step, in radians *2^24
 * 				_lAccum	rate					- rate measured at this time step, in radians/sec *2^24
 * Description:	Performs estimation step with known control inputs, but with measured angle and rate
 */
void GyroKalman::Est_Ctrl_MeasAngleAndRate(long StepTime_us,
					_lAccum rateActuation_radps2,_lAccum angle,
					_lAccum rate){
	Predict(StepTime_us,rateActuation_radps2);	// predict state estimates and estimate covariances based on past data, with known control inputs
	Update_with_Angle_and_Rate(angle,rate);		// update state estimates and covariances with new measurements (both angle and rate measurements in this case)
	return;
} // end of Est_Ctrl_MeasAngleAndRate

/*
 * Class:		GyroKalman
 * Function:	GetAngle()
 * Scope:		Public
 * Arguments:	None
 * Description:	returns estimated angle, in radians
 */
_lAccum GyroKalman::GetAngle(void){
	return angle_est;		// returns estimated angle, in radians
} // end of GetAngle()

/*
 * Class:		GyroKalman
 * Function:	GetRate()
 * Scope:		Public
 * Arguments:	None
 * Description:	returns estimated rate, in radians/sec
 */
_lAccum GyroKalman::GetRate(void){
	return rate_est;		// returns estimated rate, in radians/sec
} // end of GetRate()

/*
 * Class:		GyroKalman
 * Function:	GetEstCov()
 * Scope:		Public
 * Arguments:	byte	i	- Desired element of the estimate covariance matrix. Values:
 * 								0: first row, first column.  This is the variance of angle estimation
 * 								1: first row, second column.  This is the covariance between angle and rate estimation
 * 								2: second row, first column.  This is the covariance between rate and angle estimation
 * 								3: second row, second colunn.  This is the variance of rate estimation
 * Description:	returns the desired element of the estimate covariance matrix.
 */
_lAccum GyroKalman::GetEstCov(byte i){
	return est_cov_aposteriori[i];		// returns the desired element of the estimate covariance matrix.
} // end of GetEstCov()


// Private functions in class GyroKalman


/*
 * Class:		GyroKalman
 * Function:	Predict()
 * Scope:		private
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
 * Scope:		private
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

	if (angle_pred > PILK) angle_pred -= TWOPILK;			// if angle is greater than pi, subtract 2pi to normalize
	else if (angle_pred < -PILK) angle_pred += TWOPILK;		// else if angle is less than -pi, add 2pi to normalize

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
 * Scope:		private
 * Arguments:	None
 * Description:	performs update step without any measured data - really just skips the update step and places prediction
 * 				in estimate instead
 */
void GyroKalman::Update_with_NoData(void){
	// Update State estimates
	angle_est = angle_pred;			// set angle estimate to predicted value (no measurement)
	rate_est = rate_pred;			// set rate estimate to predicted value (no measurement)

	if (angle_est > PILK)	angle_est -= TWOPILK;			// if angle is greater than pi, subtract 2pi to normalize
	else if (angle_est < -PILK) angle_est += TWOPILK;		// else if angle is less than -pi, add 2pi to normalize

	// Update Estimate Covariance Matrix
	est_cov_aposteriori[0] = est_cov_apriori[0];	// update first row, first column of estimate covariance matrix
	est_cov_aposteriori[1] = est_cov_apriori[1];	// update first row, second column of estimate covariance matrix
	est_cov_aposteriori[2] = est_cov_apriori[2];	// update second row, first column of estimate covariance matrix
	est_cov_aposteriori[3] = est_cov_apriori[3];	// update second row, second column of estimate covariance matrix

	return;
} // end of Update_with_NoData()

/*
 * Class:		GyroKalman
 * Function:	Update_with_Angle()
 * Scope:		private
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

	if (angle_est > PILK)	angle_est -= TWOPILK;			// if angle is greater than pi, subtract 2pi to normalize
	else if (angle_est < -PILK) angle_est += TWOPILK;		// else if angle is less than -pi, add 2pi to normalize

	// Update Estimate Covariance Matrix
	est_cov_aposteriori[0] = est_cov_apriori[0] - lmullk(est_cov_apriori[0],opt_gain[0]);	// update first row, first column of estimate covariance matrix
	est_cov_aposteriori[1] = est_cov_apriori[1] - lmullk(est_cov_apriori[1],opt_gain[0]);	// update first row, second column of estimate covariance matrix
	est_cov_aposteriori[2] = est_cov_apriori[2] - lmullk(est_cov_apriori[0],opt_gain[1]);	// update second row, first column of estimate covariance matrix
	est_cov_aposteriori[3] = est_cov_apriori[3] - lmullk(est_cov_apriori[1],opt_gain[1]);	// update second row, second column of estimate covariance matrix

	return;
} // end of Update_with_Angle()

/*
 * Class:		GyroKalman
 * Function:	Update_with_Rate()
 * Scope:		private
 * Arguments:	_lAccum	rate	- rate measured at this time step, in radians/sec *2^24
 * Description:	performs update step with measured rate data (from gyro), but no angle data.
 */
void GyroKalman::Update_with_Rate(_lAccum rate){
	_lAccum meas_residual;				// measurement residual
	_lAccum residual_var;				// residual variance
	_lAccum residual_var_inv;			// inverse of residual variance;
	_lAccum opt_gain[2];				// optimal Kalman gain vector

	// calculate residual and variances
	meas_residual = rate - rate_pred;	// calculate measurement residual
	residual_var = est_cov_apriori[3] + rate_obs_noise_var;	// calculate residual variance
	residual_var_inv = ldivlk(ONE_LK,residual_var);				// calculate inverse of residual variance

	// Calculate the optimal kalman gain
	opt_gain[0] = lmullk(residual_var_inv,est_cov_apriori[1]);	// calculate first element of optimal Kalman gain
	opt_gain[1] = lmullk(residual_var_inv,est_cov_apriori[3]);	// calculate second element of optimal Kalman gain

	// Update state estimates
	angle_est = angle_pred + lmullk(opt_gain[0],meas_residual);	// update (a posteriori) angle state estimate
	rate_est = rate_pred + lmullk(opt_gain[1],meas_residual);	// update (a posteriori) rate state estimate

	if (angle_est > PILK)	angle_est -= TWOPILK;			// if angle is greater than pi, subtract 2pi to normalize
	else if (angle_est < -PILK) angle_est += TWOPILK;		// else if angle is less than -pi, add 2pi to normalize

	// Update Estimate Covariance Matrix
	est_cov_aposteriori[0] = est_cov_apriori[0] - lmullk(est_cov_apriori[2],opt_gain[0]);	// update first row, first column of estimate covariance matrix
	est_cov_aposteriori[1] = est_cov_apriori[1] - lmullk(est_cov_apriori[3],opt_gain[0]);	// update first row, second column of estimate covariance matrix
	est_cov_aposteriori[2] = est_cov_apriori[2] - lmullk(est_cov_apriori[2],opt_gain[1]);	// update second row, first column of estimate covariance matrix
	est_cov_aposteriori[3] = est_cov_apriori[3] - lmullk(est_cov_apriori[3],opt_gain[1]);	// update second row, second column of estimate covariance matrix

	return;
} // end of Update_with_Rate()

/*
 * Class:		GyroKalman
 * Function:	Update_with_Angle_and_Rate()
 * Scope:		private
 * Arguments:	_lAccum	angle	- angle measured at this time step, in radians *2^24
 * 				_lAccum	rate	- rate measured at this time step, in radians/sec *2^24
 * Description:	performs update step with both measured angle and rate data
 */
void GyroKalman::Update_with_Angle_and_Rate(_lAccum angle, _lAccum rate){
	_lAccum meas_residual[2];			// measurement residual
	_lAccum det_res_cov;				// determinant of residual covariance matrix
	_lAccum inv_det_res_cov;			// inverse of the determinant of the residual covariance matrix
	_lAccum opt_gain[4];				// optimal Kalman gain matrix
	_lAccum tempvalue;					// temporary _lAccum value used in Kalman gain calculations

	// calculate residual and variances
	meas_residual[0] = angle - angle_pred;	// calculate first element of measurement residual
	meas_residual[1] = rate - rate_pred;	// calculate second element of measurement residual

	/* In order to use less logic steps, will skip calculating the residual covariance separately,
	 * and include it directly in the steps used to calculate the optimal Kalman gain.  The
	 * determinant of the residual covariance matrix and its inverse will be calculated instead
	 * as an intermediate step, since the matrix inverse is used in Kalman gain calculation
	 */

	// calculate determinate of residual covariance matrix
	det_res_cov = 	lmullk((est_cov_apriori[0]+angle_obs_noise_var),(est_cov_apriori[3]+rate_obs_noise_var)) -
					lmullk(est_cov_apriori[1],est_cov_apriori[2]);
	inv_det_res_cov = ldivlk(ONE_LK,det_res_cov);				// calculate inverse of the determinant of the residual covariance matrix

	// calculate the optimal Kalman gain
	tempvalue = 	lmullk(est_cov_apriori[0],est_cov_apriori[3]) -
					lmullk(est_cov_apriori[1],est_cov_apriori[2]) +
					lmullk(est_cov_apriori[0],rate_obs_noise_var);	// calculate first element of optimal gain, with exception of division by determinant
	opt_gain[0] = lmullk(inv_det_res_cov,tempvalue);				// finish calculation of first element of optimal Kalman gain
	tempvalue =		lmullk(est_cov_apriori[1],angle_obs_noise_var);	// calculate second element of optimal gain, with exception of division by determinant
	opt_gain[1] = lmullk(inv_det_res_cov,tempvalue);				// finish calculation of second element of optimal Kalman gain
	tempvalue =		lmullk(est_cov_apriori[2],rate_obs_noise_var);	// calculate third element of optimal gain, with exception of division by determinant
	opt_gain[2] = lmullk(inv_det_res_cov,tempvalue);				// finish calculation of third element of optimal Kalman gain
	tempvalue = 	lmullk(est_cov_apriori[0],est_cov_apriori[3]) -
					lmullk(est_cov_apriori[1],est_cov_apriori[2]) +
					lmullk(est_cov_apriori[3],angle_obs_noise_var);	// calculate fourth element of optimal gain, with exception of division by determinant
	opt_gain[3] = lmullk(inv_det_res_cov,tempvalue);				// finish calculation of fourth element of optimal Kalman gain

	// Update state estimates
	angle_est = angle_pred + 	lmullk(opt_gain[0],meas_residual[0]) +
								lmullk(opt_gain[1],meas_residual[1]);	// update (a posteriori) angle state estimate
	rate_est = rate_pred + 		lmullk(opt_gain[3],meas_residual[0]) +
								lmullk(opt_gain[4],meas_residual[1]);	// update (a posteriori) rate state estimate

	if (angle_est > PILK)	angle_est -= TWOPILK;			// if angle is greater than pi, subtract 2pi to normalize
	else if (angle_est < -PILK) angle_est += TWOPILK;		// else if angle is less than -pi, add 2pi to normalize

	// Update Estimate Covariance Matrix
	est_cov_aposteriori[0] = 	est_cov_apriori[0] -
								lmullk(est_cov_apriori[0],opt_gain[0]) -
								lmullk(est_cov_apriori[2],opt_gain[1]);	// update first row, first column of estimate covariance matrix
	est_cov_aposteriori[1] = 	est_cov_apriori[1] -
								lmullk(est_cov_apriori[1],opt_gain[0]) -
								lmullk(est_cov_apriori[3],opt_gain[1]);	// update first row, second column of estimate covariance matrix
	est_cov_aposteriori[2] = 	est_cov_apriori[2] -
								lmullk(est_cov_apriori[0],opt_gain[2]) -
								lmullk(est_cov_apriori[2],opt_gain[3]);	// update second row, first column of estimate covariance matrix
	est_cov_aposteriori[3] = 	est_cov_apriori[3] -
								lmullk(est_cov_apriori[1],opt_gain[2]) -
								lmullk(est_cov_apriori[3],opt_gain[3]);	// update second row, second column of estimate covariance matrix
	return;
} // end of Update_with_Angle_and_Rate()
