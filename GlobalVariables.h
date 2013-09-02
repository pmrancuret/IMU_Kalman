/*
 * GlobalVariables.h
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "IMU_Kalman.h"

extern unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds

extern _lAccum	heading_rad;				// heading, in radians (*2^24)
extern int		heading_deg;				// heading, in degrees
extern _lAccum	magX_Ga;					// X-axis magnetic field, in Gauss (*2^24)
extern _lAccum	magY_Ga;					// Y-axis magnetic field, in Gauss (*2^24)
extern _lAccum	magZ_Ga;					// Z-axis magnetic field, in Gauss (*2^24)
extern byte mag_datacount;					// number of data ready in magnetometer
extern _lAccum templAccum1;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum2;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum3;					// temporary _lAccum for debugging purposes
extern _lAccum templAccum4;					// temporary _lAccum for debugging purposes
#endif /* GLOBALVARIABLES_H_ */
