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
	Serial.println("");
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
#endif



	}	// end of fastest loop
}
