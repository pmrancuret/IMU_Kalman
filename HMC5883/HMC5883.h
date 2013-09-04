/*
 * HMC5883.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Rancuret
 */

#ifndef HMC5883_H_
#define HMC5883_H_

#include <Wire.h>
#include <Arduino.h>
#include "avrfix.h"

// Sensor Sign directions
#define MAG_XAXIS_SIGN			-1		// x-axis magnetic sign convention.  setting to 1 gives default, setting to -1 gives negative
#define MAG_YAXIS_SIGN			1		// y-axis magnetic sign convention.  setting to 1 gives default, setting to -1 gives negative
#define MAG_ZAXIS_SIGN			1		// z-axis magnetic sign convention.  setting to 1 gives default, setting to -1 gives negative

// sample averaging selection options.  DO NOT CHANGE THESE except maybe the default
#define SAMPLEAVERAGING_SEL_1	0		// this selection value chooses to take one sample per measurement
#define SAMPLEAVERAGING_SEL_2	1		// this selection value chooses to take and average two samples per measurement
#define SAMPLEAVERAGING_SEL_4	2		// this selection value chooses to take and average four samples per measurement
#define SAMPLEAVERAGING_SEL_8	3		// this selection value chooses to take and average eight samples per measurement
#define SAMPLEAVERANING_SEL_DEFAULT	SAMPLEAVERAGING_SEL_8	// default selection for sample averaging

// output data rate (Hz) selection options.  DO NOT CHANGE THESE except maybe the default
#define DATAOUTPUTSEL_0_75HZ 	0		// This selection value chooses a data output rate of 0.75 Hz
#define DATAOUTPUTSEL_1_5HZ  	1		// This selection value chooses a data output rate of 1.5 Hz
#define DATAOUTPUTSEL_3HZ    	2		// This selection value chooses a data output rate of 3 Hz
#define DATAOUTPUTSEL_7_5HZ  	3		// This selection value chooses a data output rate of 7.5 Hz
#define DATAOUTPUTSEL_15HZ   	4		// This selection value chooses a data output rate of 15 Hz
#define DATAOUTPUTSEL_30HZ   	5		// This selection value chooses a data output rate of 30 Hz
#define DATAOUTPUTSEL_75HZ   	6		// This selection value chooses a data output rate of 75 Hz
#define DATAOUTPUTSEL_DEFAULT	DATAOUTPUTSEL_75HZ	// default selection for data output rate (Hz)

// measurement bias selection options.  DO NOT CHANGE THESE except maybe the default
#define NORMALOPERATION_SEL     0		// This selection value chooses normal operation - no current flow through resistive load
#define POSITIVEBIASCONFIG_SEL  1		// This selection value chooses a positive bias for the X, Y, and Z axes by forcing a positive current across each resistive load
#define NEGATIVEBIASCONFIG_SEL  2		// This selection value chooses a negative bias for the X, Y, and Z axes by forcing a negative current across each resistive load
#define BIASSEL_DEFAULT	NORMALOPERATION_SEL	// default selection for measurement bias

// magnetic range selection options.  DO NOT CHANGE THESE except maybe the default
#define MAGRANGE_SEL_0_88		0		// this selection value chooses the 0.88 Gauss full-scale range
#define MAGRANGE_SEL_1_3		1		// this selection value chooses the 1.3 Gauss full-scale range
#define MAGRANGE_SEL_1_9		2		// this selection value chooses the 1.9 Gauss full-scale range
#define MAGRANGE_SEL_2_5		3		// this selection value chooses the 2.5 Gauss full-scale range
#define MAGRANGE_SEL_4_0		4		// this selection value chooses the 4.0 Gauss full-scale range
#define MAGRANGE_SEL_4_7		5		// this selection value chooses the 4.7 Gauss full-scale range
#define MAGRANGE_SEL_5_6		6		// this selection value chooses the 5.6 Gauss full-scale range
#define MAGRANGE_SEL_8_1		7		// this selection value chooses the 8.1 Gauss full-scale range
#define MAGGAIN_SELDEFAULT		MAGRANGE_SEL_1_3	// Default selection value for magnetic range

// Address Definitions
#define COMPASS_ADDRESS			0x1E	// I2C address of the magnetometer
#define COMPASS_WRITE			0x3C	// I2C command to write data from magnetometer
#define COMPASS_READ			0x3D	// I2C command to read data from magnetometer
#define CONFIGREGA				0x00	// Config Register A
#define CONFIGREGB				0x01	// Config Register B
#define MODEREG					0x02	// Mode Register
#define DATAOUTXMSB				0x03	// X-axis data output register -  Most significant bit
#define DATAOUTXLSB				0x04	// X-axis data output register -  Least significant bit
#define DATAOUTYMSB				0x07	// Y-axis data output register -  Most significant bit
#define DATAOUTYLSB				0x08	// Y-axis data output register -  Least significant bit
#define DATAOUTZMSB				0x05	// Z-axis data output register -  Most significant bit
#define DATAOUTZLSB				0x06	// Z-axis data output register -  Least significant bit
#define STATUSREG				0x09	// Status register
#define IDENTREGA				0x0A	// Identification Register A
#define IDENTREGB				0x0B	// Identification Register B
#define IDENTREGC				0x0C	// Identification Register C

// ConfigRegA valid sample averaging
#define SAMPLEAVERAGING_1    0x00		// this bit-mask in the Config Register A causes the device to average 1 sample per measurement
#define SAMPLEAVERAGING_2    0x20		// this bit-mask in the Config Register A causes the device to average 2 sample per measurement
#define SAMPLEAVERAGING_4    0x40		// this bit-mask in the Config Register A causes the device to average 4 sample per measurement
#define SAMPLEAVERAGING_8    0x60		// this bit-mask in the Config Register A causes the device to average 8 sample per measurement

// ConfigRegA valid data output rates
#define DATAOUTPUTRATE_0_75HZ 0x00		// This bit-mask in the Config Register A causes a data output rate of 0.75 Hz
#define DATAOUTPUTRATE_1_5HZ  0x04		// This bit-mask in the Config Register A causes a data output rate of 1.5 Hz
#define DATAOUTPUTRATE_3HZ    0x08		// This bit-mask in the Config Register A causes a data output rate of 3 Hz
#define DATAOUTPUTRATE_7_5HZ  0x0C		// This bit-mask in the Config Register A causes a data output rate of 7.5 Hz
#define DATAOUTPUTRATE_15HZ   0x10		// This bit-mask in the Config Register A causes a data output rate of 15 Hz
#define DATAOUTPUTRATE_30HZ   0x14		// This bit-mask in the Config Register A causes a data output rate of 30 Hz
#define DATAOUTPUTRATE_75HZ   0x18		// This bit-mask in the Config Register A causes a data output rate of 75 Hz

// ConfigRegA valid measurement configuration bits
#define NORMALOPERATION      0x00		// This bit-mask in the Config Register A causes normal operation - no current flow through resistive load
#define POSITIVEBIASCONFIG   0x01		// This bit-mask in the Config Register A causes a positive bias for the X, Y, and Z axes by forcing a positive current across each resistive load
#define NEGATIVEBIASCONFIG   0x02		// This bit-mask in the Config Register A causes a negative bias for the X, Y, and Z axes by forcing a negative current across each resistive load

// define gain magnetic range values for ConfigRegB
#define MAGRANGE_0_88			0x00	// this bit-mask in the config register B causes the device to have a full-scale range of +- 0.88 Gauss
#define MAGRANGE_1_3			0x20	// this bit-mask in the config register B causes the device to have a full-scale range of +- 1.3 Gauss
#define MAGRANGE_1_9			0x40	// this bit-mask in the config register B causes the device to have a full-scale range of +- 1.9 Gauss
#define MAGRANGE_2_5			0x60	// this bit-mask in the config register B causes the device to have a full-scale range of +- 2.5 Gauss
#define MAGRANGE_4_0			0x80	// this bit-mask in the config register B causes the device to have a full-scale range of +- 4.0 Gauss
#define MAGRANGE_4_7			0xA0	// this bit-mask in the config register B causes the device to have a full-scale range of +- 4.7 Gauss
#define MAGRANGE_5_6			0xC0	// this bit-mask in the config register B causes the device to have a full-scale range of +- 5.6 Gauss
#define MAGRANGE_8_1			0xE0	// this bit-mask in the config register B causes the device to have a full-scale range of +- 8.1 Gauss

// define Gauss_per_LSb for each scale range
#define GAUSSPERLSB_0_88		12246	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-0.88 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_1_3			15391	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-1.3 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_1_9			20460	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-1.9 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_2_5			25420	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-2.5 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_4_0			38130	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-4.0 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_4_7			43018	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-4.7 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_5_6			50840	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-5.6 Ga.  The _lAccum data type implies it is times 2^24.
#define GAUSSPERLSB_8_1			72944	// Number of Gauss per LSb (least significant bit) of magnetic reading from the ADC with scale range of +-8.1 Ga.  The _lAccum data type implies it is times 2^24.

// ModeRegister valid modes
#define CONTINUOUSCONVERSION	0x00	// Setting this value in the Mode register causes the magnetometer to sample continuously
#define SINGLECONVERSION		0x01	// Setting this value in the mode register causes the magnetometer to take measurements one-at-a-time
#define IDLEMODE				0x02	// Setting this value in the mode register puts the magnetometer in idle mode

// constant definitions
#define MINUS128LK				0x8000	// This value represents the minimum number (-128) that can be represented using the _lAccum data type
#ifndef PIOVER2LK
#define PIOVER2LK				26353589	// This value represents pi/2 in the _lAccum data type (*2^24)
#endif
#ifndef PILK
#define PILK					52707178	// This value represents pi in the _lAccum data type (*2^24)
#endif
#ifndef TWOPILK
#define TWOPILK					105414357	// This value represents 2*pi in the _lAccum data type (*2^24)
#endif

// MPU6000 Class Definition
/*
 * Class:		HMC5883
 * Function:	NA
 * Scope:		global
 * Arguments:	NA
 * Description:	This class contains the HMC5883 functions and data.
 * 				The HMC5883 is a 3-axis magnetometer.
 */
class HMC5883 {
private:
	char			identity[3];				// 3-byte identifier of device.  must be ascii H43
	_lAccum			magX;						// magnetic reading in x-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum			magY;						// magnetic reading in y-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum			magZ;						// magnetic reading in z-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum			offsetx;					// offset for x-axis magnetic reading, in Gauss.    The _lAccum data type implies this number is times 2^24.
	_lAccum			offsety;					// offset for y-axis magnetic reading, in Gauss.    The _lAccum data type implies this number is times 2^24.
	_lAccum			offsetz;					// offset for z-axis magnetic reading, in Gauss.    The _lAccum data type implies this number is times 2^24.
	_lAccum			Gauss_per_LSb;				// this variable holds the scale of Gauss/LSb (least significant bit) of the ADC reading.  This number is based on the scale selection.
	_lAccum			heading;					// heading, in radians.  A heading of zero indicates due north, while a heading of pi indicates due south.   The _lAccum data type implies this number is times 2^24.
	_lAccum			magnetic_declination;		// magnetic declination, in radians.  This is the difference between magnetic and true north, where a positive value indicates that magnetic north is east of true north.   The _lAccum data type implies this number is times 2^24.
	volatile byte 	datacount;					// this counter increments whenever new data is available, and decrements when it is read

	boolean read(	byte startaddr,
					byte numbytes,
					byte * databuffer);			// This function reads the specified number of bytes, starting from the specified address of the HMC5883.  It places this data in the specified databuffer.
	void write(byte addr, byte data);			// This function writes the specified data at the specified address of the HMC5883

public:
	HMC5883(void);								// constructor for HMC5883 object
	boolean Initialize(void);					// This function initializes the HMC5883 device with default settings
	boolean Initialize(	byte SampleAvgSel,
						byte DataRateSel,
						byte MeasBiasSel,
						byte MagRangeSel);		// This function initializes the HMC5883 defice with specified settings
	void data_int(void);						// On Interrupt, increments counter which tells new data is ready
	void set_offset(	_lAccum offset_x,
						_lAccum offset_y,
						_lAccum offset_z);		// Sets offset values for magnetometer
	void set_mag_declination (_lAccum magdec);	// Sets the magnetic declination, in radians.  This is the difference between magnetic and true north, where a positive value indicates that magnetic north is east of true north.   The _lAccum data type implies this number is times 2^24.
	boolean Read_Mag_Data(void);				// reads magnetic data from HMC5883 magnetometer
	_lAccum Calc_Heading(_lAccum roll, _lAccum pitch);					// This function calculates and returns the heading, in radians.  A heading of zero indicates due north, while a heading of pi indicates due south.   The _lAccum data type implies this number is times 2^24.
	_lAccum Read_Mag_Data_And_Calc_Heading(_lAccum roll, _lAccum pitch);// read magnetic data and calculates and returns the heading.  returns invalid value of -128 radians if error occurred while reading data.
	char GetIdentity1(void);					// returns first character of device identifier
	char GetIdentity2(void);					// returns second character of device identifier
	char GetIdentity3(void);					// returns third character of device identifier
	_lAccum GetMagX(void);						// returns the magnetic reading in x-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum GetMagY(void);						// returns the magnetic reading in y-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum GetMagZ(void);						// returns the magnetic reading in z-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
	_lAccum GetHeading(void);					// returns the heading, in radians.  A heading of zero indicates due north, while a heading of pi indicates due south.   The _lAccum data type implies this number is times 2^24.
	_lAccum GetMagDeclination(void);			// returns the magnetic declination, in radians.  This is the difference between magnetic and true north, where a positive value indicates that magnetic north is east of true north.   The _lAccum data type implies this number is times 2^24.
	byte GetDataCount(void);					// returns the number of unread data items awaiting in the HMC5883.

};	// end of class HMC5883

// global classes
extern HMC5883 Hmc5883;

// global functions
extern void HMC5883_data_int();


#endif /* HMC5883_H_ */
