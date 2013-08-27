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

	Mpu6000.Initialize(400,SEL_DLPF_DIS,SEL_GYRO_1000,SEL_ACCEL_4);
}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}
