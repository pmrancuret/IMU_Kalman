/*
 * GlobalVariables.h
 *
 *  Created on: Aug 31, 2013
 *      Author: Rancuret
 */

#ifndef GLOBALVARIABLES_H_
#define GLOBALVARIABLES_H_

#include "avrfix.h"

extern unsigned long loopstarttime_us;		// timestamp at which the main loop started, in microseconds

extern _lAccum	heading_rad;				// heading, in radians (*2^24)
extern int		heading_deg;				// heading, in degrees

#endif /* GLOBALVARIABLES_H_ */
