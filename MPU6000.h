/*
 * MPU6000.h
 *
 *  Created on: Aug 20, 2013
 *      Author: Rancuret
 */

#ifndef MPU6000_H_
#define MPU6000_H_

// MPU6000 support for AruduIMU V3
#include <SPI.h>
#include "avrfix.h"

#define MPU6000_CHIP_SELECT_PIN 	4  // MPU6000 CHIP SELECT

// Default Sample Time
#define SAMPLETIME_DEFAULT			400		// default sample time, in Hz

// Initialization DLPFSelect Selection Options.  Do not change any values except the default!
#define SEL_DLPF_DIS				0		// this DLPFSelect option disables the digital low-pass filter
#define SEL_DLPF_188				1		// this DLPFSelect option selects a 188 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_98					2		// this DLPFSelect option selects a 98 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_42					3		// this DLPFSelect option selects a 42 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_20					4		// this DLPFSelect option selects a 20 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_10					5		// this DLPFSelect option selects a 10 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_5					6		// this DLPFSelect option selects a 5 Hz Cutoff for the digital low-pass filter
#define SEL_DLPF_DEFAULT			SEL_DLPF_DIS	// tells which option is the default

// Initialization GyroScaleSelect Selection Options.  Do not change any values except the default!
#define SEL_GYRO_250				0		// this GyroScaleSelect option selects a full scale gyro range of +- 250 deg/sec.	Resolution of 1.3315678e-4 rad/sec.  Max range of +-4.363148 rad/sec.
#define SEL_GYRO_500				1		// this GyroScaleSelect option selects a full scale gyro range of +- 500 deg/sec.	Resolution of 2.6631355e-4 rad/sec.  Max range of +-8.726296 rad/sec.
#define SEL_GYRO_1000				2		// this GyroScaleSelect option selects a full scale gyro range of +- 1000 deg/sec.	Resolution of 5.3262711e-4 rad/sec.  Max range of +-17.45259 rad/sec.
#define SEL_GYRO_2000				3		// this GyroScaleSelect option selects a full scale gyro range of +- 2000 deg/sec.	Resolution of 1.0652542e-3 rad/sec.  Max range of +-34.90518 rad/sec.
#define SEL_GYRO_DEFAULT			SEL_GYRO_1000	// tells which option is the default

// Initialization AccelScaleSelect Selection Options.  Do not change any values except the default!
#define SEL_ACCEL_2					0		// this AccelScaleSelect option selects a full scale accel range of +- 2g.	Resolution of 5.9855729e-4 m/s^2.  Max range of +-19.612927 m/s^2.
#define SEL_ACCEL_4					1		// this AccelScaleSelect option selects a full scale accel range of +- 4g.	Resolution of 1.1971146e-3 m/s^2.  Max range of +-39.225853 m/s^2.
#define SEL_ACCEL_8					2		// this AccelScaleSelect option selects a full scale accel range of +- 8g.	Resolution of 2.3942292e-3 m/s^2.  Max range of +-78.451707 m/s^2.
#define SEL_ACCEL_16				3		// this AccelScaleSelect option selects a full scale accel range of +- 16g.	Resolution of 4.7884583e-3 m/s^2.  Max range of +-128 m/s^2.
#define SEL_ACCEL_DEFAULT			SEL_ACCEL_4		// tells which option is the default

// MPU 6000 registers
#define MPUREG_WHOAMI 				0x75 	// Bits 1-6 of this register contain the upper 6 bits of the MPU-6000 I2C address
#define	MPUREG_SMPLRT_DIV 			0x19 	// Sample Rate Divider register.  Sample Rate = GyroOutRate / (1+SMPLRT_DIV), where GyroOutRate = 8 kHz when DLPF disabled, 1 kHz when DLPF is enabled
#define MPUREG_CONFIG				0x1A 	// Configures the external frame synchronization pin (FSYNC) sampling and digital low pass filter (DLPF) for both gyros and accels
#define MPUREG_GYRO_CONFIG 			0x1B	// Configures Gyroscopes.  Contains self-test mode bits, as well as full scale range selection.
#define MPUREG_ACCEL_CONFIG 		0x1C	// Configures Accelerometers.  Contains self-test mode bits, as well as full scale range selection.
#define MPUREG_INT_PIN_CFG 			0x37	//
#define	MPUREG_INT_ENABLE 			0x38	// This register enables interrupt generation by interrupt sources
#define MPUREG_ACCEL_XOUT_H			0x3B 	// Contains high byte of x-axis accelerometer measurement
#define MPUREG_ACCEL_XOUT_L			0x3C 	// Contains low byte of x-axis accelerometer measurement
#define MPUREG_ACCEL_YOUT_H			0x3D 	// Contains high byte of y-axis accelerometer measurement
#define MPUREG_ACCEL_YOUT_L			0x3E 	// Contains low byte of y-axis accelerometer measurement
#define MPUREG_ACCEL_ZOUT_H			0x3F 	// Contains high byte of z-axis accelerometer measurement
#define MPUREG_ACCEL_ZOUT_L			0x40 	// Contains low byte of z-axis accelerometer measurement
#define MPUREG_TEMP_OUT_H			0x41	// Contains high byte of temperature measurement
#define MPUREG_TEMP_OUT_L 			0x42	// Contains low byte of temperature measurement
#define MPUREG_GYRO_XOUT_H 			0x43 	// Contains high byte of x-axis gyroscope measurement
#define	MPUREG_GYRO_XOUT_L 			0x44 	// Contains low byte of x-axis gyroscope measurement
#define MPUREG_GYRO_YOUT_H 			0x45	// Contains high byte of y-axis gyroscope measurement
#define	MPUREG_GYRO_YOUT_L 			0x46 	// Contains low byte of y-axis gyroscope measurement
#define MPUREG_GYRO_ZOUT_H 			0x47 	// Contains high byte of z-axis gyroscope measurement
#define	MPUREG_GYRO_ZOUT_L 			0x48 	// Contains low byte of z-axis gyroscope measurement
#define MPUREG_USER_CTRL 			0x6A 	// Allows user to enable or disable FIFO, I2C master, and I2C mode.  Also allows resetting of these devices.
#define	MPUREG_PWR_MGMT_1 			0x6B 	// Used to configure power management.  Contains clock selection bits, reset bit, as well as sleep and cycle sleep bits.
#define	MPUREG_PWR_MGMT_2 			0x6C 	// Allows user to configure frequency of wake-ups in accelerometer only low power mode.  Also allows user to put individual axis of accel and gyro into standby.

// Configuration bits  MPU 6000

#define BIT_SLEEP 					0x40	// This bitmask in the MPUREG_PWR_MGMT_1 register will put the device in Sleep mode
#define BIT_H_RESET 				0x80	// this bitmask in the MPUREG_PWR_MGMT_1 register will reset the device
#define MPU_CLK_SEL_PLLGYROZ 		0x03	// this bitmask in the MPUREG_PWR_MGMT_1 resister will select the z-axis gyroscope clock source
#define BITS_GFS_250DPS             0x00	// This bitmask in the MPUREG_GYRO_CONFIG register will select a +- 250 degrees per second full scale range
#define BITS_GFS_500DPS             0x08	// This bitmask in the MPUREG_GYRO_CONFIG register will select a +- 500 degrees per second full scale range
#define BITS_GFS_1000DPS            0x10	// This bitmask in the MPUREG_GYRO_CONFIG register will select a +- 1000 degrees per second full scale range
#define BITS_GFS_2000DPS            0x18	// This bitmask in the MPUREG_GYRO_CONFIG register will select a +- 2000 degrees per second full scale range
#define BITS_GYRO_SELFTEST			0xE0	// This bitmask in the MPUREG_GYRO_CONFIG register will enable self-test mode for all three gyro axis'.
#define BITS_GYRO_NOSELFTEST		0x00	// This bitmask in the MPUREG_GYRO_CONFIG register will disable self-test mode for all three gyro axis' (allows normal operation).
#define BITS_AFS_2G					0x00	// This bitmask in the MPUREG_ACCEL_CONFIG register will select a +- 2g full scale range
#define BITS_AFS_4G					0x08	// This bitmask in the MPUREG_ACCEL_CONFIG register will select a +- 4g full scale range
#define BITS_AFS_8G					0x10	// This bitmask in the MPUREG_ACCEL_CONFIG register will select a +- 8g full scale range
#define BITS_AFS_16G				0x18	// This bitmask in the MPUREG_ACCEL_CONFIG register will select a +- 16g full scale range
#define BITS_ACCEL_SELFTEST			0xE0	// This bitmask in the MPUREG_ACCEL_CONFIG register will enable self-test mode for all three accel axis'.
#define BITS_ACCEL_NOSELFTEST		0x00	// This bitmask in the MPUREG_ACCEL_CONFIG register will disable self-test mode for all three accel axis' (allows normal operation).
#define MPU_NO_EXT_SYNC				0x00	// this bitmask in the MPUREG_CONFIG register will disable external synchronization
#define BITS_DLPF_CFG_256HZ_NOLPF 	0x00	// This bitmask in the MPUREG_CONFIG register will disable the low pass filter.  GyroOutRate will be 8 kHz.
#define BITS_DLPF_CFG_188HZ         0x01	// This bitmask in the MPUREG_CONFIG register will apply a 188 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define BITS_DLPF_CFG_98HZ          0x02	// This bitmask in the MPUREG_CONFIG register will apply a 98 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define BITS_DLPF_CFG_42HZ          0x03	// This bitmask in the MPUREG_CONFIG register will apply a 42 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define BITS_DLPF_CFG_20HZ          0x04	// This bitmask in the MPUREG_CONFIG register will apply a 20 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define BITS_DLPF_CFG_10HZ          0x05	// This bitmask in the MPUREG_CONFIG register will apply a 10 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define BITS_DLPF_CFG_5HZ           0x06	// This bitmask in the MPUREG_CONFIG register will apply a 5 Hz LPF to measurements.  GyroOutRate will be 1 kHz.
#define	BIT_INT_ANYRD_2CLEAR	    0x10	// This bitmask in the MPUREG_INT_PIN_CFG register will allow interrupt status to be cleared on any read operation.
#define	BIT_RAW_RDY_EN		    	0x01	// This bitmask in the MPUREG_INT_ENABLE register will enable the 'DATA_RDY_EN' interrupt, which tell when new raw data is ready to read.
#define	BIT_I2C_IF_DIS              0x10	// this bitmask in the MPUREG_USER_CTRL register will disable the I2C module

#define MAX_RADPSPCNT_LK			17872	// This value represents the number of rad/sec for each gyroscope count, times 2^24.  Max means it is the value with the +-2000 deg/sec full scale resolution.
#define MAX_MPS2PCNT_LK				80337	// This value represents the number of m/s^2 for each accelerometer count, times 2^24.  Max means it is the value with the +-16g full scale resolution.

// MPU6000 Class Definition
/*
 * Class:		MPU6000
 * Function:	NA
 * Scope:		global
 * Arguments:	NA
 * Description:	This class contains the MPU600 functions and data.
 * 				The MPU6000 is a 3-axis accelerometer and gyroscope.
 */
class MPU6000 {
private:
	_lAccum			accelX;					// X-axis acceleration, m/s^2.  The data type _lAccum implies it is stored with a factor of 2^24.
	_lAccum 		accelY;					// Y-axis acceleration, m/s^2.  The data type _lAccum implies it is stored with a factor of 2^24.
	_lAccum 		accelZ;					// Z-axis acceleration, m/s^2.  The data type _lAccum implies it is stored with a factor of 2^24.
	_lAccum 		gyroX;					// X-axis gyroscopic rate, scale*rad/sec.  The data type _lAccum implies it is stored with a factor of 2^24.
	_lAccum 		gyroY;					// Y-axis gyroscopic rate, scale*rad/sec.  The data type _lAccum implies it is stored with a factor of 2^24.
	_lAccum 		gyroZ;					// Z-axis gyroscopic rate, scale*rad/sec.  The data type _lAccum implies it is stored with a factor of 2^24.
	_sAccum			temp;					// temperature of MPU6000, deg C.  The data type _sAccum implies it is stored with a factor of 2^8.
	volatile byte 	datacount;				// this counter increments whenever new data is available, and decrements when it is read
	byte 			identity;				// stores the identity code of the MPU6000 processor
	byte			DLPF_Select;			// stores the Digital Low Pass Filter (DLPF) cutoff frequency selection
	byte			Gyro_Select;			// stores the gyroscope scale selection
	byte			Accel_Select;			// stores the accelerometer scale selection

	void SPI_write(byte reg, byte data);	// Writes to a register in the MPU6000 using SPI
	byte SPI_read(byte reg);				// reads a register in the MPU6000 using SPI

public:
	MPU6000(void);							// Constructor for MPU6000 object
	boolean Initialize(void);				// This function initializes the MPU6000 object using default settings
	boolean Initialize(	unsigned int SampleRate,
						byte DLPFSelect,
						byte GyroScaleSelect,
						byte AccelScaleSelect);		// This function initializes the MPU6000 object using settings supplied in argument
	void Read_Accel_and_Gyro(void);			// This function reads all accelerometer and gyro data
	void Read_Accel(void);					// This function reads all accelerometer data
	void Read_Gyro(void);					// This function reads all gyroscope data
	void Read_Temp(void);					// This function reads temperature data
	void Read_All_Data(void);				// This function reads all gyroscope, accelerometer, and temperature data
	void data_int(void);					// On Interrupt, increments counter which tells new data is ready
	_lAccum GetAccelX(void);				// returns latest read x-axis accelerometer measurement (does not perform a new read)
	_lAccum GetAccelY(void);				// returns latest read y-axis accelerometer measurement (does not perform a new read)
	_lAccum GetAccelZ(void);				// returns latest read z-axis accelerometer measurement (does not perform a new read)
	_lAccum GetGyroX(void);					// returns latest read x-axis gyroscope measurement (does not perform a new read)
	_lAccum GetGyroY(void);					// returns latest read y-axis gyroscope measurement (does not perform a new read)
	_lAccum GetGyroZ(void);					// returns latest read z-axis gyroscope measurement (does not perform a new read)
	_sAccum GetTemp(void);					// returns latest read temperature measurement (does not perform a new read)
	byte GetDataCount(void);				// returns number of unread data items waiting in the MPU6000
	byte GetIdentity(void);					// returns identity of the MPU6000 device

} ;	// end of class MPU6000

// global classes
extern MPU6000 Mpu6000;

// global variables

//Sensor variables

// global functions
extern void MPU6000_data_int();



#endif /* MPU6000_H_ */
