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
	Hmc5883.Initialize(	SAMPLEAVERAGING_SEL_8,
						DATAOUTPUTSEL_75HZ,
						NORMALOPERATION_SEL,
						MAGRANGE_SEL_1_3);			// initialize HMC5883 with 8 sample averaging, 75 Hz data rate, normal (no bias) operation, and +-1.3 gauss scale
	Hmc5883.set_offset(	MAG_OFFSET_X,
						MAG_OFFSET_Y,
						MAG_OFFSET_Z);				// set magnetic sensor offset values
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}
