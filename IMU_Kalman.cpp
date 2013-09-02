/*
 * Imu_Kalman.cpp
 *
 *  Created on: Aug 26, 2013
 *      Author: Rancuret
 */

// Do not remove the include below
#include "Imu_Kalman.h"

//The setup function is called once at startup of the sketch
void setup()
{
// Add your initialization code here
	Serial.begin(SERIAL_DATARATE);

	if (Mpu6000.Initialize(	400,
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
	Mpu6000.Set_Gyro_Offsets(	GYRO_OFFSET_X,
								GYRO_OFFSET_Y,
								GYRO_OFFSET_Z);		// set gyroscope offsets
	Mpu6000.Set_Accel_Offsets(	ACCEL_OFFSET_X,
								ACCEL_OFFSET_Y,
								ACCEL_OFFSET_Z);	// set accelerometer offsets
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
								XGYRO_ANGLEESTVAR,
								XGYRO_RATEESTVAR);	// Initialize the x-axis gyroscopic kalman filter

	YAxisGyroKalman.Initialize(	YGYRO_ANGLEPROCVAR,
								YGYRO_RATEPROCVAR,
								YGYRO_ANGLEOBSVAR,
								YGYRO_RATEOBSVAR,
								YGYRO_INITIALANGLE,
								YGYRO_INITIALRATE,
								YGYRO_ANGLEESTVAR,
								YGYRO_RATEESTVAR);	// Initialize the y-axis gyroscopic kalman filter

	ZAxisGyroKalman.Initialize(	ZGYRO_ANGLEPROCVAR,
								ZGYRO_RATEPROCVAR,
								ZGYRO_ANGLEOBSVAR,
								ZGYRO_RATEOBSVAR,
								ZGYRO_INITIALANGLE,
								ZGYRO_INITIALRATE,
								ZGYRO_ANGLEESTVAR,
								ZGYRO_RATEESTVAR);	// Initialize the z-axis gyroscopic kalman filter
	Serial.println("");
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
	if (micros() - loopstarttime_us >= FASTLOOP_TIME_US)	// if fast loop sample time has elapsed
	{
		loopdeltatime_us = micros() - loopstarttime_us;			// store amount of time elapsed since beginning last loop
		loopstarttime_us = micros();							// begin new loop iteration, and record new starting time

#ifdef DEBUG_MAGNETOMETER
		Debug_Mag_Messages();	// print magnetometer debug messages
#endif

		CheckForData();					// check for new data
		Calculate_Kalman_Estimates();	// Calculate all Kalman filter state estimates

#ifdef DEBUG_PRINTFILTEROUTS
		Print_Filter_Debug_Out();		// print filter debugging output messages
#endif


	}	// end of fastest loop
}
