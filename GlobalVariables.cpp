/*
 * GlobalVariables.cpp
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */
#include "GlobalVariables.h"

unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds

_lAccum	heading_rad;				// heading, in radians (*2^24)
int		heading_deg;				// heading, in degrees
