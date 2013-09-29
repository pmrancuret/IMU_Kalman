/*
 * utilities.cpp
 *
 *  Created on: Sep 2, 2013
 *      Author: Rancuret
 */

#include "utilities.h"

// class instantiations
CalcMeanAndVariance XAxisAngleNoise;		// this is used to calculate the x-axis angle measurement mean (rad) and variance (rad^2)
CalcMeanAndVariance YAxisAngleNoise;		// this is used to calculate the y-axis angle measurement mean (rad) and variance (rad^2)
CalcMeanAndVariance ZAxisAngleNoise;		// this is used to calculate the z-axis angle measurement mean (rad) and variance (rad^2)
CalcMeanAndVariance XAxisRateNoise;			// this is used to calculate the x-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)
CalcMeanAndVariance YAxisRateNoise;			// this is used to calculate the y-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)
CalcMeanAndVariance ZAxisRateNoise;			// this is used to calculate the z-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)
CalcMeanAndVariance XAxisAccelNoise;		// this is used to calculate the x-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)
CalcMeanAndVariance YAxisAccelNoise;		// this is used to calculate the y-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)
CalcMeanAndVariance ZAxisAccelNoise;		// this is used to calculate the z-axis rate measurement mean (rad/sec) and variance (rad^2/sec^2)


/*
 * Class:		NA
 * Function:	Initialize_System()
 * Scope:		NA
 * Arguments:	None
 * Description:	Initializes the IMU system
 */
void Initialize_System(void){
	_lAccum XAxisAccelMean = 0;				// x-axis accelerometer mean
	_lAccum YAxisAccelMean = 0;				// y-axis accelerometer mean
	_lAccum ZAxisAccelMean = 0;				// z-axis accelerometer mean
	_lAccum XAxisAngleVar = 0;				// x-axis angle variance
	_lAccum YAxisAngleVar = 0;				// y-axis angle variance
	_lAccum ZAxisAngleVar = 0;				// z-axis angle variance
	_lAccum XAxisRateMean = 0;				// x-axis rate mean
	_lAccum YAxisRateMean = 0;				// y-axis rate mean
	_lAccum ZAxisRateMean = 0;				// z-axis rate mean
	_lAccum XAxisRateVar = 0;				// x-axis rate variance
	_lAccum YAxisRateVar = 0;				// y-axis rate variance
	_lAccum ZAxisRateVar = 0;				// z-axis rate variance
	long numiterations = 0;					// number of iterations ran in attempt to calculate means and variances

	// Begin Serial Communications
	Serial.begin(SERIAL_DATARATE);

	// Initialize the MPU6000 gyro/accel
	if (Mpu6000.Initialize(	MPU_SAMPLERATE_HZ,
						SEL_DLPF_DIS,
						SEL_GYRO_1000,
						SEL_ACCEL_4))				// initialize MPU6000 with 400 Hz sample rate, no low pass filter, +-1000 dps gyro scale, and +- 4g accel scale
	{
		Serial.println("Succesfully Initialized the MPU6000");
	}
	else
	{
		Serial.println("Failed to Initialize the MPU6000");
	}

	// Initialize the HMC5883 magnetometer
	if (Hmc5883.Initialize(	SAMPLEAVERAGING_SEL_8,
						DATAOUTPUTSEL_75HZ,
						NORMALOPERATION_SEL,
						MAGRANGE_SEL_1_3))			// initialize HMC5883 with 8 sample averaging, 75 Hz data rate, normal (no bias) operation, and +-1.3 gauss scale
	{
		Serial.println("Succesfully Initialized the HMC5883");
	}
	else
	{
		Serial.println("Failed to Initialize the HMC5883");
	}

	// Enter loop to calculate means and variances of measurements
	while (numiterations < NUMBER_INITSAMPLES)
	{
		if (micros() - loopstarttime_us >= FASTLOOP_TIME_US)	// if fast loop sample time has elapsed
		{
			loopdeltatime_us = micros() - loopstarttime_us;		// store amount of time elapsed since beginning last loop
			loopstarttime_us = micros();						// begin new loop iteration, and record new starting time
			numiterations++;									// increment the number of iterations passed

			CheckForData();										// check for new measured data

			if (newmagdata)										// if new magnetometer data was read
			{
				ZAxisAngleNoise.CalcNextValue(heading_rad,&ZAxisAngleVar);	// calculate variance of yaw measurement
			}

			if (newmpudata)										// if new IMU data was read
			{
				XAxisAngleNoise.CalcNextValue(roll_fromAccel,&XAxisAngleVar);				// calculate variance of roll angle measurement
				YAxisAngleNoise.CalcNextValue(pitch_fromAccel,&YAxisAngleVar);				// calculate variance of pitch angle measurement
				XAxisAccelMean = XAxisAccelNoise.CalcNextValue(accelX_meas,NULL);			// calculate mean of  x-axis accelerometer measurement
				YAxisAccelMean = YAxisAccelNoise.CalcNextValue(accelY_meas,NULL);			// calculate mean of  y-axis accelerometer measurement
				ZAxisAccelMean = ZAxisAccelNoise.CalcNextValue(accelZ_meas,NULL);			// calculate mean of  z-axis accelerometer measurement
				XAxisRateMean = XAxisRateNoise.CalcNextValue(rollrate_meas,&XAxisRateVar);	// calculate mean and variance of x-axis rate measurement
				YAxisRateMean = YAxisRateNoise.CalcNextValue(pitchrate_meas,&YAxisRateVar);	// calculate mean and variance of y-axis rate measurement
				ZAxisRateMean = ZAxisRateNoise.CalcNextValue(yawrate_meas,&ZAxisRateVar);	// calculate mean and variance of z-axis rate measurement
			}

		}

	}

	Mpu6000.Set_Gyro_Offsets(	-XAxisRateMean,
								-YAxisRateMean,
								-ZAxisRateMean);		// set gyroscope offsets
	Mpu6000.Set_Accel_Offsets(	-XAxisAccelMean,
								-YAxisAccelMean,
								-(ZAxisAccelMean+GRAVITYMPS2_LK));		// set accelerometer offsets
	Hmc5883.set_offset(	MAG_OFFSET_X,
						MAG_OFFSET_Y,
						MAG_OFFSET_Z);				// set magnetic sensor offset values
	Hmc5883.set_mag_declination(MAG_DECLINATION);	// set magnetic declination (difference between true and magnetic north)

	XAxisGyroKalman.Initialize(	XGYRO_ANGLEPROCVAR,
								XGYRO_RATEPROCVAR,
								XGYRO_ANGLEOBSVAR,
								XGYRO_RATEOBSVAR,
								XGYRO_INITIALANGLE,
								XGYRO_INITIALRATE,
								XAxisAngleVar,
								XAxisRateVar);	// Initialize the x-axis gyroscopic kalman filter

	YAxisGyroKalman.Initialize(	YGYRO_ANGLEPROCVAR,
								YGYRO_RATEPROCVAR,
								YGYRO_ANGLEOBSVAR,
								YGYRO_RATEOBSVAR,
								YGYRO_INITIALANGLE,
								YGYRO_INITIALRATE,
								YAxisAngleVar,
								YAxisRateVar);	// Initialize the y-axis gyroscopic kalman filter

	ZAxisGyroKalman.Initialize(	ZGYRO_ANGLEPROCVAR,
								ZGYRO_RATEPROCVAR,
								ZGYRO_ANGLEOBSVAR,
								ZGYRO_RATEOBSVAR,
								ZGYRO_INITIALANGLE,
								ZGYRO_INITIALRATE,
								ZAxisAngleVar,
								ZAxisRateVar);	// Initialize the z-axis gyroscopic kalman filter
	Serial.println("Finished Initialization");
	Serial.println("");
	return;
}	// end of Initialize_System()


/*
 * Class:		NA
 * Function:	CheckForData()
 * Scope:		NA
 * Arguments:	None
 * Description:	Checks for and reads data from MPU and magnetometer
 */
void CheckForData(void){
	if (Hmc5883.GetDataCount() > 0)								// if new data is available from magnetometer
	{
		if (Hmc5883.Read_Mag_Data())							// read magnetic data
		{
			if ( (roll < FIVEDEGREES_LK) && (roll > -FIVEDEGREES_LK) && (pitch < FIVEDEGREES_LK) && (pitch > -FIVEDEGREES_LK) ) // if roll and pitch are suitably small
			{
				heading_rad = Hmc5883.Calc_Heading(roll,pitch);		// calculate heading
				newmagdata = true;									// denote new data ready
			}
			else
			{
				newmagdata = false;
			}
		}
		else													// if failed to read data
		{
			newmagdata = false;									// denote no new data ready
		}
	}
	else														// no data was available yet
	{
		newmagdata = false;										// denote no new data was ready
	}

	if (Mpu6000.GetDataCount() > 0)								// if new data is available from MPU
	{
		Mpu6000.Read_Accel_and_Gyro();							// read the new accelerometer and gyro data from SPI bus
		rollrate_meas = Mpu6000.GetGyroX();						// aircraft roll rate about x-axis, in radians/sec.  a positive value indicates increasing roll towards right.  measured from IMU
		pitchrate_meas = Mpu6000.GetGyroY();					// aircraft pitch rate about y-axis, in radians/sec.  a positive value indicates increasing pitch upwards.  measured from IMU
		yawrate_meas = Mpu6000.GetGyroZ();						// aircraft yaw rate about z-axis, in radians/sec.  a positive value indicates increasing yaw right.  measured from IMU
		accelX_meas = Mpu6000.GetAccelX();						// measured acceleration in x-direction, m/s^2.  measured from IMU
		accelY_meas = Mpu6000.GetAccelY();						// measured acceleration in y-direction, m/s^2.  measured from IMU
		accelZ_meas = Mpu6000.GetAccelZ();						// measured acceleration in z-direction, m/s^2.  measured from IMU
		roll_fromAccel = Mpu6000.GetAngleX();					// calculated roll angle, using accelerometer data (rad)
		pitch_fromAccel = Mpu6000.GetAngleY();					// calculated pitch angle, using accelerometer data (rad)
		newmpudata = true;										// denote that new data was read from MPU
	}
	else
	{
		newmpudata = false;										// denote no new data read from MPU
	}
	return;
} // end of CheckForData

/*
 * Class:		NA
 * Function:	Calculate_Kalman_Estimates()
 * Scope:		NA
 * Arguments:	None
 * Description:	calculates all kalman state estimates, based on whatever new measurements are ready
 */
void Calculate_Kalman_Estimates(void){
	if (newmagdata)					// if new magnetometer data was ready
	{
		if (newmpudata)				// if new magnetometer and mpu data are both ready
		{
			XAxisGyroKalman.Est_NoCtrl_MeasAngleAndRate(loopdeltatime_us,roll_fromAccel,rollrate_meas);		// trigger x-axis gyro kalman estimation
			YAxisGyroKalman.Est_NoCtrl_MeasAngleAndRate(loopdeltatime_us,pitch_fromAccel,pitchrate_meas);	// trigger y-axis gyro kalman estimation
			ZAxisGyroKalman.Est_NoCtrl_MeasAngleAndRate(loopdeltatime_us,heading_rad,yawrate_meas);			// trigger z-axis gyro kalman estimation
		}
		else						// if new magnetometer data is ready, but no mpu data
		{
			XAxisGyroKalman.Est_NoCtrl_NoMeas(loopdeltatime_us);											// trigger x-axis gyro kalman estimation
			YAxisGyroKalman.Est_NoCtrl_NoMeas(loopdeltatime_us);											// trigger y-axis gyro kalman estimation
			ZAxisGyroKalman.Est_NoCtrl_MeasAngle(loopdeltatime_us,heading_rad);								// trigger z-axis gyro kalman estimation
		}
	}
	else							// if no magnetometer data is ready
	{
		if (newmpudata)				// if no magnetometer data, but new mpu data is ready
		{
			XAxisGyroKalman.Est_NoCtrl_MeasAngleAndRate(loopdeltatime_us,roll_fromAccel,rollrate_meas);		// trigger x-axis gyro kalman estimation
			YAxisGyroKalman.Est_NoCtrl_MeasAngleAndRate(loopdeltatime_us,pitch_fromAccel,pitchrate_meas);	// trigger y-axis gyro kalman estimation
			ZAxisGyroKalman.Est_NoCtrl_MeasRate(loopdeltatime_us,yawrate_meas);								// trigger z-axis gyro kalman estimation
		}
		else						// if no magnetometer data or mpu data is ready
		{
			XAxisGyroKalman.Est_NoCtrl_NoMeas(loopdeltatime_us);											// trigger x-axis gyro kalman estimation
			YAxisGyroKalman.Est_NoCtrl_NoMeas(loopdeltatime_us);											// trigger y-axis gyro kalman estimation
			ZAxisGyroKalman.Est_NoCtrl_NoMeas(loopdeltatime_us);											// trigger z-axis gyro kalman estimation
		}
	}

	roll = XAxisGyroKalman.GetAngle();			// get estimated roll angle (rad)
	pitch = YAxisGyroKalman.GetAngle();			// get estimated pitch angle (rad)
	yaw = ZAxisGyroKalman.GetAngle();			// get estimated yaw angle (rad)
	rollrate = XAxisGyroKalman.GetRate();		// get estimated roll rate (rad/sec)
	pitchrate = YAxisGyroKalman.GetRate();		// get estimated pitch rate (rad/sec)
	yawrate = ZAxisGyroKalman.GetRate();		// get estimated yaw rate (rad/sec)

	return;
}	// end of Calculate_Kalman_Estimates()

#ifdef DEBUG_MAGNETOMETER
/*
 * Class:		NA
 * Function:	Debug_Mag_Messages()
 * Scope:		NA
 * Arguments:	None
 * Description:	prints magnetometer data for debugging purposes
 */
void Debug_Mag_Messages(void){
	if (Hmc5883.GetDataCount() > 0)							// if new data is available from magnetometer
	{
		mag_datacount = Hmc5883.GetDataCount();						// get number of data items ready
		if (Hmc5883.Read_Mag_Data())									// read magnetic data
		{
			magX_Ga = Hmc5883.GetMagX();								// get x-axis magnetic field strength, in Gauss (*2^24)
			magY_Ga = Hmc5883.GetMagY();								// get y-axis magnetic field strength, in Gauss (*2^24)
			magZ_Ga = Hmc5883.GetMagZ();								// get z-axis magnetic field strength, in Gauss (*2^24)
			heading_rad = Hmc5883.Calc_Heading(0,0);					// calculate heading
			heading_deg = (int)(((heading_rad>>15)*((long)29335))>>18);	// heading, in degrees

			Serial.println("Data Count\t\tX-Axis Mag(Ga*2^8)\t\tY-Axis Mag(Ga*2^8)\t\tZ-Axis Mag (Ga*2^8)\t\tHeading (deg)");
			Serial.print(mag_datacount,DEC);
			Serial.print("\t\t\t\t\t\t\t\t");
			Serial.print(magX_Ga>>16,DEC);
			Serial.print("\t\t\t\t\t\t\t\t\t\t");
			Serial.print(magY_Ga>>16,DEC);
			Serial.print("\t\t\t\t\t\t\t\t\t\t");
			Serial.print(magZ_Ga>>16,DEC);
			Serial.print("\t\t\t\t\t\t\t\t\t");
			Serial.println(heading_deg,DEC);
			Serial.println("");
		}
		else
		{
			Serial.println("Error reading magnetic data");
		}

	}
	else
	{
		Serial.println("No Magnetic Data Ready");
	}
	return;
} // end of Debug_Mag_Messages
#endif

#ifdef DEBUG_PRINTFILTEROUTS
/*
 * Class:		NA
 * Function:	Print_Filter_Debug_Out()
 * Scope:		NA
 * Arguments:	None
 * Description:	print filter debugging output messages
 */
void Print_Filter_Debug_Out(void){
	static byte i = 0;			// counter variable

	if (++i >= 25)				// if this is the tenth time this function has been called
	{
		i = 0;					// reset counter

		Serial.println("TimeStep (us)\t\tRaw Rollrate (deg/s)\t\tRaw Pitchrate (deg/s)\t\tRaw Yawrate (deg/s)\t\tRaw Roll (deg)\t\tRaw Pitch (deg)\t\tRaw Yaw (deg)\t\tFilt Rollrate (deg/s)\t\tFilt Pitchrate (deg/s)\t\tFilt Yawrate (deg/s)\t\tFilt Roll (deg)\t\tFilt Pitch (deg)\t\tFilt Yaw (deg)");
		Serial.print(loopdeltatime_us,DEC);
		Serial.print("\t\t\t\t\t\t");
		Serial.print((int)(((rollrate_meas>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((pitchrate_meas>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((yawrate_meas>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((roll_fromAccel>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t");
		Serial.print((int)(((pitch_fromAccel>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t");
		Serial.print((int)(((heading_rad>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t");
		Serial.print((int)(((rollrate>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((pitchrate>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((yawrate>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((roll>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t");
		Serial.print((int)(((pitch>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((yaw>>15)*((long)29335))>>18),DEC);

		Serial.println("");
		Serial.println("");
	}
	return;		// print filter debugging output messages
} // end of Print_Filter_Debug_Out()
#endif

/*
 * Class:		CalcMeanAndVariance
 * Function:	CalcMeanAndVariance
 * Scope:		public
 * Arguments:	None
 * Description:	constructor for CalcMeanAndVariance class
 */
CalcMeanAndVariance::CalcMeanAndVariance(void){
	n = 1;			// initialize number of samples taken to one
	mean = 0;		// initialize mean to zero
	variance = 0;	// initialize variance to zero
	M2 = 0;			// initialize M2 to zero
	return;
} // end of CalcMeanAndVariance()

/*
 * Class:		CalcMeanAndVariance
 * Function:	CalcNextValue
 * Scope:		public
 * Arguments:	None
 * Description:	calculates the next value for mean and variance
 */
_lAccum CalcMeanAndVariance::CalcNextValue(_lAccum measured, _lAccum * variance_out){
	_lAccum delta;				// change between this value and the current mean

	n++;						// increment number of samples
	delta = measured - mean;	// calculate change between this measurement and last mean
	mean += delta/n;		// calculate new mean
	M2 += lmullk(delta,(measured - mean));
	variance = M2/(n-1);
	*variance_out = variance;
	return mean;
}  // end of CalcNextValue()

/*
 * Class:		CalcMeanAndVariance
 * Function:	ReInitialize
 * Scope:		public
 * Arguments:	_lAccum	mean_init	- initial value for mean
 * 				_lAccum var_init	- initial value for variance
 * Description:	initializes the mean and variance values
 */
void CalcMeanAndVariance::Initialize(_lAccum mean_init, _lAccum var_init){
	n = 1;					// initialize number of samples taken to one
	mean = mean_init;		// initialize mean to specified value
	variance = var_init;	// initialize variance to specified value
	M2 = 0;					// initialize M2 to zero
	return;
} // end of Initialize()
