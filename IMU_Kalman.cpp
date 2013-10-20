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
	Initialize_System();			// initialize the IMU
	delay(5);						// delay 5 ms
	loopstarttime_us = micros();	// reset loop start timer
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

#ifndef DEBUG_NOBINARYDATA
		PrintBinaryData();				// print state output message for autopilot to read
#endif

#ifdef DEBUG_PRINTFILTEROUTS
		Print_Filter_Debug_Out();		// print filter debugging output messages
#endif

	}	// end of fastest loop
}
