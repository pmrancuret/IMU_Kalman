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

	Mpu6000.Initialize(	400,
						SEL_DLPF_DIS,
						SEL_GYRO_1000,
						SEL_ACCEL_4);				// initialize MPU6000 with 400 Hz sample rate, no low pass filter, +-1000 dps gyro scale, and +- 4g accel scale
	Mpu6000.Set_Gyro_Offsets(	GYRO_OFFSET_X,
								GYRO_OFFSET_Y,
								GYRO_OFFSET_Z);		// set gyroscope offsets
	Mpu6000.Set_Accel_Offsets(	ACCEL_OFFSET_X,
								ACCEL_OFFSET_Y,
								ACCEL_OFFSET_Z);	// set accelerometer offsets
	Hmc5883.Initialize(	SAMPLEAVERAGING_SEL_8,
						DATAOUTPUTSEL_75HZ,
						NORMALOPERATION_SEL,
						MAGRANGE_SEL_1_3);			// initialize HMC5883 with 8 sample averaging, 75 Hz data rate, normal (no bias) operation, and +-1.3 gauss scale
	Hmc5883.set_offset(	MAG_OFFSET_X,
						MAG_OFFSET_Y,
						MAG_OFFSET_Z);				// set magnetic sensor offset values
	Hmc5883.set_mag_declination(MAG_DECLINATION);	// set magnetic declination (difference between true and magnetic north)
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
	if (micros() - loopstarttime_us >= FASTLOOP_TIME_US)	// if fast loop sample time has elapsed
	{
	loopstarttime_us = micros();							// begin new loop iteration, and record new starting time

#ifdef DEBUG_MAGNETOMETER
	if (Hmc5883.GetDataCount() > 0)							// if new data is available from magnetometer
	{
		heading_rad = Hmc5883.Read_Mag_Data_And_Calc_Heading(0,0);	// read magnetic data and calculate heading

		heading_deg = (int)(((heading_rad>>15)*((long)29335))>>18);	// heading, in degrees
	}
#endif

	}	// end of fastest loop
}
