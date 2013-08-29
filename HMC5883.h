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
#define CONFIGREGA				0x00	// Config Register A
#define CONFIGREGB				0x01	// Config Register B
#define MODEREG					0x02	// Mode Register
#define DATAOUTXMSB				0x03	// X-axis data output register -  Most significant bit
#define DATAOUTXLSB				0x04	// X-axis data output register -  Least significant bit
#define DATAOUTYMSB				0x05	// Y-axis data output register -  Most significant bit
#define DATAOUTYLSB				0x06	// Y-axis data output register -  Least significant bit
#define DATAOUTZMSB				0x07	// Z-axis data output register -  Most significant bit
#define DATAOUTZLSB				0x08	// Z-axis data output register -  Least significant bit
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

// ModeRegister valid modes
#define CONTINUOUSCONVERSION	0x00	// Setting this value in the Mode register causes the magnetometer to sample continuously
#define SINGLECONVERSION		0x01	// Setting this value in the mode register causes the magnetometer to take measurements one-at-a-time
#define IDLEMODE				0x02	// Setting this value in the mode register puts the magnetometer in idle mode

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
	char	identity[3];						// 3-byte identifier of device.  must be ascii H43
	int		offsetx;							// offset for x-axis reading
	int		offsety;							// offset for y-axis reading
	int		offsetz;							// offset for z-axis reading
	int		magx_counts;						// magnetic reading in x-axis, in ADC reading counts
	int		magy_counts;						// magnetic reading in x-axis, in ADC reading counts
	int		magz_counts;						// magnetic reading in x-axis, in ADC reading counts

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

};	// end of class HMC5883

// global classes
extern HMC5883 Hmc5883;

#endif /* HMC5883_H_ */
