/*
 * MPU6000.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: Rancuret
 *
 *      Description:	MPU6000 support for ArduIMU V3
 */

#include "MPU6000.h"

// definitions
MPU6000 Mpu6000;


// MPU6000 public functions

/*
 * Class:		MPU6000
 * Function:	MPU6000()
 * Scope:		Public
 * Arguments:	None
 * Description:	Constructor for MPU6000 object
 */
MPU6000::MPU6000(){
	accelX = 0;							// Initial value of 0.  X-axis acceleration, scale*m/s^2 (full scale range depends on configuration)
	accelY = 0;							// Initial value of 0.  Y-axis acceleration, scale*m/s^2 (full scale range depends on configuration)
	accelZ = 0;							// Initial value of 0.  Z-axis acceleration, scale*m/s^2 (full scale range depends on configuration)
	gyroX = 0;							// Initial value of 0.  X-axis gyroscopic rate, scale*rad/sec (full scale range depends on configuration)
	gyroY = 0;							// Initial value of 0.  Y-axis gyroscopic rate, scale*rad/sec (full scale range depends on configuration)
	gyroZ = 0;							// Initial value of 0.  Z-axis gyroscopic rate, scale*rad/sec (full scale range depends on configuration)
	temp = 0;							// Initial value of 0.  temperature of MPU6000, degC*2^8
	datacount = 0;						// Initial value of 0.  this counter increments whenever new data is available, and decrements when it is read
	identity = 0;						// Initial value of 0.  stores the identity code of the MPU6000 processor
	DLPF_Select = SEL_DLPF_DEFAULT;		// Initial value of default.  stores the Digital Low Pass Filter (DLPF) cutoff frequency selection
	Gyro_Select = SEL_GYRO_DEFAULT;		// Initial value of default.  stores the gyroscope scale selection
	Accel_Select = SEL_ACCEL_DEFAULT;	// Initial value of default.  stores the accelerometer scale selection
	return;
}

/*
 * Class:		MPU6000
 * Function:	Initialize()
 * Scope:		Public
 * Arguments:	None
 * Description:	This function initializes the MPU6000 object using default settings
 * 				Defaults:  	No Digital Lowpass Filter (bandwidth of 256 Hz)
 * 							Sample Rate = 400 Hz
 * 							Gyro Scale = full scale of +- 1000 deg/sec.
 * 								Yields digital units of 3.0518044e-2 deg/sec/count,
 * 								or 5.3264035e-4 rad/sec/count.
 * 							Accel Scale = full scale of +- 4g.
 * 								Yields digital units of 1.2207218e-4 g/count,
 * 								or 1.1971191e-3 m/s^2/count.
 */
boolean MPU6000::Initialize(void){

	return Initialize(	SAMPLETIME_DEFAULT,
						SEL_DLPF_DEFAULT,
						SEL_GYRO_DEFAULT,
						SEL_ACCEL_DEFAULT);

}

/*
 * Class:		MPU6000
 * Function:	Initialize()
 * Scope:		Public
 * Arguments:	unsigned int SampleRate	Selects Sampling Rate of MPU6000 measurement, in Hz.
 * 											When DLPF is enabled, this must be evenly divisible into 1000
 * 											When DLPF is disabled, this must be evenly divisible into 8000
 * 				byte	DLPFSelect		Selects Frequency of Digital Low-Pass Filter, if enabled.
 * 										Values:	SEL_DLPF_DIS - DLPF Disabled.
 * 												SEL_DLPF_188 - 188 Hz Cutoff.
 * 												SEL_DLPF_98 - 98 Hz Cutoff.
 * 												SEL_DLPF_42 - 42 Hz Cutoff.
 * 												SEL_DLPF_20 - 20 Hz Cutoff.
 * 												SEL_DLPF_10 - 10 Hz Cutoff.
 * 												SEL_DLPF_5 - 5 Hz Cutoff.
 * 												anything else - disables DLPF and returns false
 * 				byte	GyroScaleSelect	Selects full scale measurement range of gyroscopes
 * 										Values:	SEL_GYRO_250 - Full Scale range of +- 250 deg/sec.
 * 												SEL_GYRO_500 - Full Scale range of +- 500 deg/sec.
 * 												SEL_GYRO_1000 - Full Scale range of +- 1000 deg/sec.
 * 												SEL_GYRO_2000 - Full Scale range of +- 2000 deg/sec.
 * 												anything else - sets full scale range of +- 1000 deg/sec and returns false
 * 				byte	AccelScaleSelect Selects full scale measurement range of accelerometers
 * 										Values: SEL_ACCEL_2 - Full Scale range of +- 2g.
 * 												SEL_ACCEL_4 - Full Scale range of +- 4g.
 * 												SEL_ACCEL_8 - Full Scale range of +- 8g.
 * 												SEL_ACCEL_16 - Full Scale range of +- 16g.
 * 												anything else - sets full scale range of +- 4g and returns false.
 * Description:	This function initializes the MPU6000 object using settings supplied in argument
 */
boolean MPU6000::Initialize(unsigned int SampleRate, byte DLPFSelect, byte GyroScaleSelect, byte AccelScaleSelect){
	boolean rtn = true;	// value to return at end
	byte SmplRtDiv;		// Sample Rate Divisor value to be stored in MPUREG_SMPLRT_DIV register

	DLPF_Select = DLPFSelect;				// store value of DLPF cutoff
	Gyro_Select = GyroScaleSelect;			// store value of gyroscope scale
	Accel_Select = AccelScaleSelect;		// store value of accelerometer scale

	// MPU6000 chip select setup
	pinMode(MPU6000_CHIP_SELECT_PIN, OUTPUT);
	digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);

	// SPI initialization
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
	delay(10);

	// Chip reset
	SPI_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	delay(100);

	// Wake up device and select GyroZ clock (better performance)
	SPI_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	delay(1);

	// Disable I2C bus (recommended on datasheet)
	SPI_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
	delay(1);

	// Get identity of the device, 'cause why not.  Note this identifier is only really used for I2C comms which we aren't using.  But here it is.
	identity = (SPI_read(MPUREG_WHOAMI) & 0x7E);

	// Set No External Sync, and select appropriate DLPF cutoff frequency
	switch (DLPFSelect)
	{
	case SEL_DLPF_DIS:	// DLPF disabled, GyroOutRate = 8 kHz.  Also, calculate sample rate divisor value using SmplRtDiv= GyroOutRate/SampleRate - 1.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_256HZ_NOLPF);	// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)8000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_188:	// DLPF cutoff 188 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_188HZ);		// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_98:	// DLPF cutoff 98 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_98HZ);			// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_42:	// DLPF cutoff 42 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_42HZ);			// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_20:	// DLPF cutoff 20 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_20HZ);			// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_10:	// DLPF cutoff 10 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_10HZ);			// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	case SEL_DLPF_5:	// DLPF cutoff 5 Hz, GyroOutRate = 1 kHz.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | BITS_DLPF_CFG_5HZ);			// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)1000/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		break;
	default:			// DLPF disabled, GyroOutRate = 8 kHz.  return false.
		SPI_write(MPUREG_CONFIG, MPU_NO_EXT_SYNC | SEL_DLPF_DEFAULT );	// set bits of config register
		SmplRtDiv = (byte)constrain((unsigned int)(SEL_DLPF_DEFAULT == SEL_DLPF_DIS ? 8000:1000)/SampleRate,1,255) - 1;	// calculate sample rate divisor value
		DLPF_Select = SEL_DLPF_DEFAULT;
		rtn = false;
	}
	delay(1);

	// SAMPLE RATE
	SPI_write(MPUREG_SMPLRT_DIV,SmplRtDiv);     // Set desired sample rate
	delay(1);

	// Set Gyroscope Scale and ensure it is not in self-test mode
	switch (GyroScaleSelect)
	{
	case SEL_GYRO_250: 	// Gyro scale +-250º/s
		SPI_write(MPUREG_GYRO_CONFIG, BITS_GYRO_NOSELFTEST | BITS_GFS_250DPS);  // Set bits of MPUREG_GYRO_CONFIG register to disable self-test and apply Gyro scale 250º/s
		break;
	case SEL_GYRO_500:	// Gyro scale +-500º/s
		SPI_write(MPUREG_GYRO_CONFIG, BITS_GYRO_NOSELFTEST | BITS_GFS_500DPS);  // Set bits of MPUREG_GYRO_CONFIG register to disable self-test and apply Gyro scale 500º/s
		break;
	case SEL_GYRO_1000:	// Gyro scale +-1000º/s
		SPI_write(MPUREG_GYRO_CONFIG, BITS_GYRO_NOSELFTEST | BITS_GFS_1000DPS);  // Set bits of MPUREG_GYRO_CONFIG register to disable self-test and apply Gyro scale 1000º/s
		break;
	case SEL_GYRO_2000:	// Gyro scale +-2000º/s
		SPI_write(MPUREG_GYRO_CONFIG, BITS_GYRO_NOSELFTEST | BITS_GFS_2000DPS);  // Set bits of MPUREG_GYRO_CONFIG register to disable self-test and apply Gyro scale 2000º/s
		break;
	default:			// Gyro scale +-1000º/s and return false
		SPI_write(MPUREG_GYRO_CONFIG, BITS_GYRO_NOSELFTEST | (SEL_GYRO_DEFAULT<<3));  // Set bits of MPUREG_GYRO_CONFIG register to disable self-test and apply Gyro scale 1000º/s
		Gyro_Select = SEL_GYRO_DEFAULT;		// store default selection
		rtn = false;
	}
	delay(1);

	// Set Accelerometer Scale and nsure it is not in self-test mode
	switch (AccelScaleSelect)
	{
	case SEL_ACCEL_2:	// Accel scale +-2g
		SPI_write(MPUREG_ACCEL_CONFIG, BITS_ACCEL_NOSELFTEST | BITS_AFS_2G);      // Set bits of MPUREG_ACCEL_CONFIG register to disable self-test and apply Accel scale 2g
		break;
	case SEL_ACCEL_4:	// Accel scale +-4g
		SPI_write(MPUREG_ACCEL_CONFIG, BITS_ACCEL_NOSELFTEST | BITS_AFS_4G);      // Set bits of MPUREG_ACCEL_CONFIG register to disable self-test and apply Accel scale 4g
		break;
	case SEL_ACCEL_8:	// Accel scale +-8g
		SPI_write(MPUREG_ACCEL_CONFIG, BITS_ACCEL_NOSELFTEST | BITS_AFS_8G);      // Set bits of MPUREG_ACCEL_CONFIG register to disable self-test and apply Accel scale 8g
		break;
	case SEL_ACCEL_16:	// Accel scale +-16g
		SPI_write(MPUREG_ACCEL_CONFIG, BITS_ACCEL_NOSELFTEST | BITS_AFS_16G);      // Set bits of MPUREG_ACCEL_CONFIG register to disable self-test and apply Accel scale 16g
		break;
	default:			// Accel scale +-4g and return false
		SPI_write(MPUREG_ACCEL_CONFIG, BITS_ACCEL_NOSELFTEST | (SEL_ACCEL_DEFAULT<<3));      // Set bits of MPUREG_ACCEL_CONFIG register to disable self-test and apply Accel scale 4g
		Accel_Select = SEL_ACCEL_DEFAULT;	// store default selection
		rtn = false;
	}
	delay(1);

	// INT CFG => Interrupt on Data Ready
	SPI_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
	delay(1);
	SPI_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
	delay(1);

	// MPU_INT is connected to INT 0. Enable interrupt on INT0
	attachInterrupt(0,MPU6000_data_int,RISING);

	return rtn;
}

/*
 * Class:		MPU6000
 * Function:	Read_Accel_and_Gyro()
 * Scope:		public
 * Arguments:	none
 * Description:	This function reads all accelerometer and gyro data
 */
void MPU6000::Read_Accel_and_Gyro(void){
	int byte_H;
	int byte_L;

	// Read AccelX
	byte_H = SPI_read(MPUREG_ACCEL_XOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_XOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelX = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelY
	byte_H = SPI_read(MPUREG_ACCEL_YOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_YOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelY = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelZ
	byte_H = SPI_read(MPUREG_ACCEL_ZOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_ZOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelZ = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read GyroX
	byte_H = SPI_read(MPUREG_GYRO_XOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_XOUT_L);
	gyroX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroY
	byte_H = SPI_read(MPUREG_GYRO_YOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_YOUT_L);
	gyroY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroZ
	byte_H = SPI_read(MPUREG_GYRO_ZOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_ZOUT_L);
	gyroZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	return;
}

/*
 * Class:		MPU6000
 * Function:	Read_Accel()
 * Scope:		public
 * Arguments:	none
 * Description:	This function reads all accelerometer data
 */
void MPU6000::Read_Accel(void){
	int byte_H;
	int byte_L;

	// Read AccelX
	byte_H = SPI_read(MPUREG_ACCEL_XOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_XOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelX = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelY
	byte_H = SPI_read(MPUREG_ACCEL_YOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_YOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelY = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelZ
	byte_H = SPI_read(MPUREG_ACCEL_ZOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_ZOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelZ = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	return;
}

/*
 * Class:		MPU6000
 * Function:	Read_Gyro()
 * Scope:		public
 * Arguments:	none
 * Description:	This function reads all gyro data
 */
void MPU6000::Read_Gyro(void){
	int byte_H;
	int byte_L;

	// Read GyroX
	byte_H = SPI_read(MPUREG_GYRO_XOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_XOUT_L);
	gyroX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroY
	byte_H = SPI_read(MPUREG_GYRO_YOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_YOUT_L);
	gyroY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroZ
	byte_H = SPI_read(MPUREG_GYRO_ZOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_ZOUT_L);
	gyroZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	return;
}

/*
 * Class:		MPU6000
 * Function:	Read_All_Data()
 * Scope:		public
 * Arguments:	none
 * Description:	This function reads all accelerometer, gyro, and temperature data
 */
void MPU6000::Read_All_Data(void){
	int byte_H;
	int byte_L;
	int32_t	temp32int;

	// Read AccelX
	byte_H = SPI_read(MPUREG_ACCEL_XOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_XOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelX = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelY
	byte_H = SPI_read(MPUREG_ACCEL_YOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_YOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelY = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read AccelZ
	byte_H = SPI_read(MPUREG_ACCEL_ZOUT_H);
	byte_L = SPI_read(MPUREG_ACCEL_ZOUT_L);
	if (Accel_Select == SEL_ACCEL_16)	// if using the largest scale, the measurement needs to be constrained
	{
		accelZ = (_lAccum)((constrain((long)(int)((byte_H<<8)| byte_L),-26730,26730))*((long)MAX_MPS2PCNT_LK));
	}
	else
	{
		accelZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_MPS2PCNT_LK>>(SEL_ACCEL_16-Accel_Select)));
	}

	// Read GyroX
	byte_H = SPI_read(MPUREG_GYRO_XOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_XOUT_L);
	gyroX = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroY
	byte_H = SPI_read(MPUREG_GYRO_YOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_YOUT_L);
	gyroY = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

	// Read GyroZ
	byte_H = SPI_read(MPUREG_GYRO_ZOUT_H);
	byte_L = SPI_read(MPUREG_GYRO_ZOUT_L);
	gyroZ = (_lAccum)(((long)(int)((byte_H<<8)| byte_L))*((long)MAX_RADPSPCNT_LK>>(SEL_GYRO_2000-Gyro_Select)));

    // Read Temp
    byte_H = SPI_read(MPUREG_TEMP_OUT_H);
    byte_L = SPI_read(MPUREG_TEMP_OUT_L);
    temp32int = constrain((int32_t)((byte_H<<8)| byte_L),-32767,31098);
    temp = (_sAccum)((temp32int * 1542)>>11) + 9352;	// temp in degC*256 = temp32int*256/340 +  256*36.53.  = temp32int*1542/2048 + 9352.   From datasheet

	return;
}

/*
 * Class:		MPU6000
 * Function:	Read_Temp()
 * Scope:		public
 * Arguments:	none
 * Description:	This function reads  temperature data
 */
void MPU6000::Read_Temp(void){
	int byte_H;
	int byte_L;
	int32_t	temp32int;

	// Read Temp
    byte_H = SPI_read(MPUREG_TEMP_OUT_H);
    byte_L = SPI_read(MPUREG_TEMP_OUT_L);
    temp32int = constrain((int32_t)((byte_H<<8)| byte_L),-32767,31098);
    temp = (_sAccum)(((temp32int * 1542)>>11) + 9352);	// temp in degC*256 = temp32int*256/340 +  256*36.53.  = temp32int*1542/2048 + 9352.   From datasheet

	return;
}

/*
 * Class:		MPU6000
 * Function:	data_int()
 * Scope:		public
 * Arguments:	none
 * Description:	Increments counter which tells new data is ready
 */
void MPU6000::data_int(void) {
	datacount++;
	return;
}

/*
 * Class:		MPU6000
 * Function:	GetAccelX()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read x-axis accelerometer measurement (does not perform a new read)
 */
_lAccum MPU6000::GetAccelX(void) {
	return accelX;
}

/*
 * Class:		MPU6000
 * Function:	GetAccelY()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read y-axis accelerometer measurement (does not perform a new read)
 */
_lAccum MPU6000::GetAccelY(void) {
	return accelY;
}

/*
 * Class:		MPU6000
 * Function:	GetAccelZ()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read z-axis accelerometer measurement (does not perform a new read)
 */
_lAccum MPU6000::GetAccelZ(void) {
	return accelZ;
}

/*
 * Class:		MPU6000
 * Function:	GetGyroX()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read x-axis gyroscopic measurement (does not perform a new read)
 */
_lAccum MPU6000::GetGyroX(void) {
	return gyroX;
}

/*
 * Class:		MPU6000
 * Function:	GetGyroY()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read y-axis gyroscopic measurement (does not perform a new read)
 */
_lAccum MPU6000::GetGyroY(void) {
	return gyroY;
}

/*
 * Class:		MPU6000
 * Function:	GetGyroZ()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read z-axis gyroscopic measurement (does not perform a new read)
 */
_lAccum MPU6000::GetGyroZ(void) {
	return gyroZ;
}

/*
 * Class:		MPU6000
 * Function:	GetTemp()
 * Scope:		public
 * Arguments:	none
 * Description:	returns latest read temperature measurement (does not perform a new read)
 */
_sAccum MPU6000::GetTemp(void) {
	return temp;
}

/*
 * Class:		MPU6000
 * Function:	GetDataCount()
 * Scope:		public
 * Arguments:	none
 * Description:	returns number of unread data items waiting in the MPU6000
 */
byte MPU6000::GetDataCount(void) {
	return datacount;
}

/*
 * Class:		MPU6000
 * Function:	GetIdentity()
 * Scope:		public
 * Arguments:	none
 * Description:	returns identity of the MPU6000
 */
byte MPU6000::GetIdentity(void) {
	return identity;
}

// MPU6000 private SPI functions

/*
 * Class:		MPU6000
 * Function:	SPI_write()
 * Scope:		private
 * Arguments:	byte reg 	- MPU6000 register to write to
 * 				byte data	- data to write in that register
 * Description:	Writes to a register in the MPU6000 using SPI
 */
void MPU6000::SPI_write(byte reg, byte data){
	  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);	// set chip select low to enable SPI comms with MPU6000
	  SPI.transfer(reg);						// first transmit the register to write to
	  SPI.transfer(data);					// then transmit the data to write
	  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);	// return chip select high to disable SPI comms with MPU6000
	  return;
}

/*
 * Class:		MPU6000
 * Function:	SPI_read()
 * Scope:		private
 * Arguments:	byte reg 	- MPU6000 register to write to
 * Description:	reads a register in the MPU6000 using SPI
 */
byte MPU6000::SPI_read(byte reg){
  byte return_value;		// value to return
  byte addr = reg | 0x80; 	// Set most significant bit to signify this is a read operation

  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);	// set chip select low to enable SPI comms with MPU6000
  SPI.transfer(addr);							// first transmit the register to write to
  return_value = SPI.transfer(0);				// then receive the data from that register
  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);	// return chip select high to disable SPI comms with MPU6000
  return(return_value);
}

// Global Functions

/*
 * Class:		NA
 * Function:	MPU6000_data_int()
 * Scope:		global
 * Arguments:	none
 * Description:	MPU6000 INTERRUPT ON INT0 - calls function data_int() in class Mpu6000
 */
 void MPU6000_data_int(void)
 {
	 Mpu6000.data_int();
	 return;
 }

