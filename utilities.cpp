/*
 * utilities.cpp
 *
 *  Created on: Sep 2, 2013
 *      Author: Rancuret
 */

#include "utilities.h"


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
			heading_rad = Hmc5883.Calc_Heading(roll,pitch);		// calculate heading
			newmagdata = true;									// denote new data ready
		}
		else													// if failed to read data
		{
			Serial.println("Error reading magnetic data");		// print failure message
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

	if (++i >= 50)				// if this is the tenth time this function has been called
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
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((yawrate>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((roll>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((pitch>>15)*((long)29335))>>18),DEC);
		Serial.print("\t\t\t\t\t\t\t\t\t");
		Serial.print((int)(((yaw>>15)*((long)29335))>>18),DEC);

		Serial.println("");
		Serial.println("");
	}
	return;		// print filter debugging output messages
} // end of Print_Filter_Debug_Out()
#endif
