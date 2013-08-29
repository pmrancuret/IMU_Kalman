/*
 * HMC5883.cpp
 *
 *  Created on: Aug 25, 2013
 *      Author: Rancuret
 *      Description:	Provides support for HMC5883 on ArduIMU version 3.
 *      				The HMC5883 is a 3-axis magnetometer.
 */


#include "HMC5883.h"

// definitions
HMC5883 Hmc5883;

// HMC5883 public functions

/*
 * Class:		HMC5883
 * Function:	HMC5883()
 * Scope:		Public
 * Arguments:	None
 * Description:	Constructor for HMC5883 object
 */
HMC5883::HMC5883(void) {
	identity[0] = 0;					// initialize to zero.  first byte of identifier of device.  must be ascii H when read
	identity[1] = 0;					// initialize to zero.  second byte of identifier of device.  must be ascii 4 when read
	identity[2] = 0;					// initialize to zero.  third byte of identifier of device.  must be ascii 3 when read
	offsetx = 0;						// initialize to zero.  offset for x-axis reading
	offsety = 0;						// initialize to zero.  offset for y-axis reading
	offsetz = 0;						// initialize to zero.  offset for z-axis reading
	magx_counts = 0;					// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	magy_counts = 0;					// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	magz_counts = 0;					// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	return;
}

/*
 * Class:		HMC5883
 * Function:	Initialize()
 * Scope:		Public
 * Arguments:	None
 * Description:	This function initializes the HMC5883 device with default settings
 */
boolean HMC5883::Initialize(void) {
	return Initialize(	SAMPLEAVERANING_SEL_DEFAULT,
						DATAOUTPUTSEL_DEFAULT,
						BIASSEL_DEFAULT,
						MAGGAIN_SELDEFAULT);	// initialize with default values
}

/*
 * Class:		HMC5883
 * Function:	Initialize()
 * Scope:		Public
 * Arguments:	None
 * Description:	This function initializes the HMC5883 device
 */
boolean HMC5883::Initialize(byte SampleAvgSel, byte DataRateSel, byte MeasBiasSel, byte MagRangeSel) {
	boolean rtn = true;							// value to return at end of function
	byte	write_value;						// this byte temporarily stores info to transmit to magnetometer

	Wire.begin();								// initialize and begin the I2C communications line
	delay(10);									// wait 10 ms

	// read the identifier letters
	if (!read(IDENTREGA,3,(byte*)&identity))	// read identity bytes from HMC3883
	{
		rtn = false;							// if failed to succesfully read, set return value to false, indicating failure
	}

	// check to ensure identifier bytes match expected values for the HMC3883
	if (	(identity[0] != 'H') |				// if the first identifier byte is not 'H'
			(identity[1] != '4') |				// or the second identifier byte is not '4'
			(identity[2] != '3'))				// or the third identifier byte is not '3'
	{
		rtn = false;							// set return value to false, indicating failure
	}

	// prepare to write configuration selections in config register A
	switch (SampleAvgSel)						// determine which bits to set for selected sample average setting
	{
	case SAMPLEAVERAGING_SEL_1:					// if selected 1 sample
		write_value = SAMPLEAVERAGING_1;		// set correct bits for 1 sample
		break;
	case SAMPLEAVERAGING_SEL_2:					// if selected 2 samples
		write_value = SAMPLEAVERAGING_2;		// set correct bits for 2 samples
		break;
	case SAMPLEAVERAGING_SEL_4:					// if selected 4 samples
		write_value = SAMPLEAVERAGING_4;		// set correct bits for 4 samples
		break;
	case SAMPLEAVERAGING_SEL_8:					// if selected 8 samples
		write_value = SAMPLEAVERAGING_8;		// set correct bits for 8 samples
		break;
	default:									// if an invalid value is specified
		write_value = SAMPLEAVERANING_SEL_DEFAULT << 5;	// set bits for default value
		rtn = false;							// set return value to false, indicating failure with selection
	}

	switch (DataRateSel)						// determine which bits to set for selected data rate setting
	{
	case DATAOUTPUTSEL_0_75HZ:					// if selected 0.75 Hz data rate
		write_value |= DATAOUTPUTRATE_0_75HZ;	// set correct bits for 0.75 Hz data rate
		break;
	case DATAOUTPUTSEL_1_5HZ:					// if selected 1.5 Hz data rate
		write_value |= DATAOUTPUTRATE_1_5HZ;	// set correct bits for 1.5 Hz data rate
		break;
	case DATAOUTPUTSEL_3HZ:						// if selected 3 Hz data rate
		write_value |= DATAOUTPUTRATE_3HZ;		// set correct bits for 3 Hz data rate
		break;
	case DATAOUTPUTSEL_7_5HZ:					// if selected 7.5 Hz data rate
		write_value |= DATAOUTPUTRATE_7_5HZ;	// set correct bits for 7.5 Hz data rate
		break;
	case DATAOUTPUTSEL_15HZ:					// if selected 15 Hz data rate
		write_value |= DATAOUTPUTRATE_15HZ;		// set correct bits for 15 Hz data rate
		break;
	case DATAOUTPUTSEL_30HZ:					// if selected 30 Hz data rate
		write_value |= DATAOUTPUTRATE_30HZ;		// set correct bits for 30 Hz data rate
		break;
	case DATAOUTPUTSEL_75HZ:					// if selected 75 Hz data rate
		write_value |= DATAOUTPUTRATE_75HZ;		// set correct bits for 75 Hz data rate
		break;
	default:									// if an invalid value is specified
		write_value |= DATAOUTPUTSEL_DEFAULT << 2;	// set bits for default value
		rtn = false;							// set return value to false, indicating failure with selection
	}

	switch (MeasBiasSel)						// determine which bits to set for selected measurement configuration bits
	{
	case NORMALOPERATION_SEL:					// if selected normal operation
		write_value |= NORMALOPERATION;			// set correct bits for normal operation
		break;
	case POSITIVEBIASCONFIG_SEL:				// if selected operation with positive bias
		write_value |= POSITIVEBIASCONFIG;		// set correct bits for operation with positive bias
		break;
	case NEGATIVEBIASCONFIG_SEL:				// if selected operation with negative bias
		write_value |= NEGATIVEBIASCONFIG;		// set correct bits for operation with negative bias
		break;
	default:									// if invalid value is specified
		write_value |= BIASSEL_DEFAULT;			// set bits for default value
		rtn = false;							// set return value to fale, indicating failure with selection
	}

	write(CONFIGREGA,write_value);				// write specified value in Config Register A
	delay(50);									// wait 50 ms

	// prepare to write selections in config register B
	switch (MagRangeSel)						// determine which bits to set for magnetic range selection
	{
	case MAGRANGE_SEL_0_88:						// if selected +-0.88 Gauss full-scale range
		write_value = MAGRANGE_0_88;			// set correct bits for +-0.88 Gauss full-scale range
		break;
	case MAGRANGE_SEL_1_3:						// if selected +-1.3 Gauss full-scale range
		write_value = MAGRANGE_1_3;				// set correct bits for +-1.3 Gauss full-scale range
		break;
	case MAGRANGE_SEL_1_9:						// if selected +-1.9 Gauss full-scale range
		write_value = MAGRANGE_1_9;				// set correct bits for +-1.9 Gauss full-scale range
		break;
	case MAGRANGE_SEL_2_5:						// if selected +-2.5 Gauss full-scale range
		write_value = MAGRANGE_2_5;				// set correct bits for +-2.5 Gauss full-scale range
		break;
	case MAGRANGE_SEL_4_0:						// if selected +-4.0 Gauss full-scale range
		write_value = MAGRANGE_4_0;				// set correct bits for +-4.0 Gauss full-scale range
		break;
	case MAGRANGE_SEL_4_7:						// if selected +-4.7 Gauss full-scale range
		write_value = MAGRANGE_4_7;				// set correct bits for +-4.7 Gauss full-scale range
		break;
	case MAGRANGE_SEL_5_6:						// if selected +-5.6 Gauss full-scale range
		write_value = MAGRANGE_5_6;				// set correct bits for +-5.6 Gauss full-scale range
		break;
	case MAGRANGE_SEL_8_1:						// if selected +-8.1 Gauss full-scale range
		write_value = MAGRANGE_8_1;				// set correct bits for +-8.1 Gauss full-scale range
		break;
	default:									// if invalid value is specified
		write_value = MAGGAIN_SELDEFAULT << 5;	// set bits for default value
		rtn = false;							// set return value to fale, indicating failure with selection
}

	write(CONFIGREGB,write_value);				// write specified value in config register B
	delay(50);									// wait 50 ms

	// prepare to write selections in mode register
	write_value = CONTINUOUSCONVERSION;			// set mode to continuous conversion
	write(MODEREG,write_value);					// write specified value in mode register
	delay(50);									// wait 50 ms

	return rtn;									// return
}



// HMC5883 private functions

/*
 * Class:		HMC5883
 * Function:	read()
 * Scope:		private
 * Arguments:	None
 * Description:	This function reads the specified number of bytes, starting from the specified
 * 				address of the HMC5883.  It places this data in the specified databuffer.
 */
boolean HMC5883::read(byte startaddr, byte numbytes, byte * databuffer) {
	boolean rtn = true;	// value to return (true or false for success or failure)
	byte i = 0;			// initialize index used for counting received bytes

	Wire.beginTransmission(COMPASS_ADDRESS);		// Begin I2C transmission with the magnetometer
	Wire.write(startaddr);							// send starting address to read from
	Wire.endTransmission();							// end I2C transmission

	Wire.requestFrom((byte)COMPASS_ADDRESS,numbytes);		// request specified number of bytes from HMC5883
	while(Wire.available())							// while data is available on I2C bus
	{
		*(databuffer+i) = Wire.read();				// receive one byte
		i++;										// increment counter of received bytes
	}
	Wire.endTransmission();							// end I2C transmission

	if (i != numbytes)								// if number of bytes received does not match request
	{
		rtn = false;								// set return to false to indicate failure
	}
	return rtn;										// return
} // end of read()

/*
 * Class:		HMC5883
 * Function:	write()
 * Scope:		private
 * Arguments:	None
 * Description:	This function writes the specified data at the specified address of the HMC5883
 */
void HMC5883::write(byte addr, byte data) {
	Wire.beginTransmission(COMPASS_ADDRESS);	// Begin I2C transmission with the magnetometer
	Wire.write(addr);							// Tell magnetometer which register we are writing to
	Wire.write(data);							// write specified value to register
	Wire.endTransmission();						// end transmission
	return;
}
