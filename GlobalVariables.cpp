/*
 * GlobalVariables.cpp
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */
#include "GlobalVariables.h"

unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds

_lAccum	heading_rad = 0;			// heading, in radians (*2^24)
int		heading_deg = 0;			// heading, in degrees
_lAccum	magX_Ga = 0;				// X-axis magnetic field, in Gauss (*2^24)
_lAccum	magY_Ga = 0;				// Y-axis magnetic field, in Gauss (*2^24)
_lAccum	magZ_Ga = 0;				// Z-axis magnetic field, in Gauss (*2^24)
byte mag_datacount = 0;				// number of data ready in magnetometer
