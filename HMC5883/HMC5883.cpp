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
	magX = 0;							// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	magY = 0;							// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	magZ = 0;							// initialize to zero.  magnetic reading in x-axis, in ADC reading counts
	magnetic_declination = 0;			// initialize to zero.  magnetic declination, in radians.  This is the difference between magnetic and true north, where a positive value indicates that magnetic north is east of true north.   The _lAccum data type implies this number is times 2^24.
	heading = 0;						// magnetic heading, in radians.  A heading of zero indicates due north, while a heading of pi indicates due south.
#if MAGGAIN_SELDEFAULT == MAGRANGE_SEL_0_88	// if default range is set to +-0.88Ga
	Gauss_per_LSb = GAUSSPERLSB_0_88;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_1_3// if default range is set to +-1.3Ga
	Gauss_per_LSb = GAUSSPERLSB_1_3;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_1_9// if default range is set to +-1.9Ga
	Gauss_per_LSb = GAUSSPERLSB_1_9;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_2_5// if default range is set to +-2.5Ga
	Gauss_per_LSb = GAUSSPERLSB_2_5;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_4_0// if default range is set to +-4.0Ga
	Gauss_per_LSb = GAUSSPERLSB_4_0;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_4_7// if default range is set to +-4.7Ga
	Gauss_per_LSb = GAUSSPERLSB_4_7;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_5_6// if default range is set to +-5.6Ga
	Gauss_per_LSb = GAUSSPERLSB_5_6;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_8_1// if default range is set to +-8.1Ga
	Gauss_per_LSb = GAUSSPERLSB_8_1;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#else										// if default range isn't a valid value
	Gauss_per_LSb = 0;						// set zero scale factor - no measurement.  This would imply the user set an invalid default value and an invalid initialization setting
#endif
	datacount = 0;						// Initial value of 0.  this counter increments whenever new data is available, and decrements when it is read
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
		Gauss_per_LSb = GAUSSPERLSB_0_88;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_1_3:						// if selected +-1.3 Gauss full-scale range
		write_value = MAGRANGE_1_3;				// set correct bits for +-1.3 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_1_3;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_1_9:						// if selected +-1.9 Gauss full-scale range
		write_value = MAGRANGE_1_9;				// set correct bits for +-1.9 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_1_9;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_2_5:						// if selected +-2.5 Gauss full-scale range
		write_value = MAGRANGE_2_5;				// set correct bits for +-2.5 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_2_5;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_4_0:						// if selected +-4.0 Gauss full-scale range
		write_value = MAGRANGE_4_0;				// set correct bits for +-4.0 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_4_0;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_4_7:						// if selected +-4.7 Gauss full-scale range
		write_value = MAGRANGE_4_7;				// set correct bits for +-4.7 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_4_7;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_5_6:						// if selected +-5.6 Gauss full-scale range
		write_value = MAGRANGE_5_6;				// set correct bits for +-5.6 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_5_6;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	case MAGRANGE_SEL_8_1:						// if selected +-8.1 Gauss full-scale range
		write_value = MAGRANGE_8_1;				// set correct bits for +-8.1 Gauss full-scale range
		Gauss_per_LSb = GAUSSPERLSB_8_1;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
		break;
	default:									// if invalid value is specified
		write_value = MAGGAIN_SELDEFAULT << 5;	// set bits for default value
#if MAGGAIN_SELDEFAULT == MAGRANGE_SEL_0_88		// if default range is set to +-0.88Ga
		Gauss_per_LSb = GAUSSPERLSB_0_88;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_1_3	// if default range is set to +-1.3Ga
		Gauss_per_LSb = GAUSSPERLSB_1_3;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_1_9	// if default range is set to +-1.9Ga
		Gauss_per_LSb = GAUSSPERLSB_1_9;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_2_5	// if default range is set to +-2.5Ga
		Gauss_per_LSb = GAUSSPERLSB_2_5;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_4_0	// if default range is set to +-4.0Ga
		Gauss_per_LSb = GAUSSPERLSB_4_0;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_4_7	// if default range is set to +-4.7Ga
		Gauss_per_LSb = GAUSSPERLSB_4_7;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_5_6	// if default range is set to +-5.6Ga
		Gauss_per_LSb = GAUSSPERLSB_5_6;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#elif MAGGAIN_SELDEFAULT == MAGRANGE_SEL_8_1	// if default range is set to +-8.1Ga
		Gauss_per_LSb = GAUSSPERLSB_8_1;		// set correct scale factor of Gauss per LSb (least significant bit).  The _lAccum data type implies it is times 2^24
#else											// if default range isn't a valid value
		Gauss_per_LSb = 0;						// set zero scale factor - no measurement.  This would imply the user set an invalid default value and an invalid initialization setting
#endif
		rtn = false;							// set return value to fale, indicating failure with selection
}

	write(CONFIGREGB,write_value);				// write specified value in config register B
	delay(50);									// wait 50 ms

	// prepare to write selections in mode register
	write_value = CONTINUOUSCONVERSION;			// set mode to continuous conversion
	write(MODEREG,write_value);					// write specified value in mode register
	delay(50);									// wait 50 ms

	// DRDY is connected to INT 1 on pin D3. Enable interrupt on INT1
	attachInterrupt(1,HMC5883_data_int,RISING);

	return rtn;									// return
}

/*
 * Class:		HMC5883
 * Function:	data_int()
 * Scope:		public
 * Arguments:	none
 * Description:	Increments counter which tells new data is ready
 */
void HMC5883::data_int(void) {
	datacount++;
	return;
}

/*
 * Class:		HMC5883
 * Function:	set_offset()
 * Scope:		public
 * Arguments:	none
 * Description:	Sets offset values for magnetometer
 */
void HMC5883::set_offset(_lAccum offset_x, _lAccum offset_y, _lAccum offset_z)
{
	offsetx = offset_x;
	offsetx = offset_y;
	offsetx = offset_z;
}

/*
 * Class:		HMC5883
 * Function:	set_mag_declination()
 * Scope:		public
 * Arguments:	none
 * Description:	Sets the magnetic declination, in radians.  This is the difference
 * 				between magnetic and true north, where a positive value indicates
 * 				that magnetic north is east of true north.   The _lAccum data type
 * 				implies this number is times 2^24.
 */
void HMC5883::set_mag_declination (_lAccum magdec){
	magnetic_declination = magdec;					// set the magnetic declination as proscribed
	return;
}


/*
 * Class:		HMC5883
 * Function:	Read_Mag_Data()
 * Scope:		public
 * Arguments:	none
 * Description: reads magnetic data from HMC5883 magnetometer
 */
boolean HMC5883::Read_Mag_Data(void){				// reads magnetic data from HMC5883 magnetometer
	boolean rtn = true;								// value to return at end - initialize to true
	byte databuffer[6];								// buffer to store measured data in
	int mag_counts;									// temporary int holds digital (unscaled) counts from x-axis magnetic measurement

	if (!read(DATAOUTXMSB,6,&databuffer[0]))		// read data
	{
		rtn = false;								// if read failure, set return value to false to indicate the failure
	}
	else
	{
		mag_counts = ( ((int)databuffer[0]) << 8 | databuffer[1] ) * MAG_XAXIS_SIGN;	// read x-axis adc counts
		if ((mag_counts > 2047) | (mag_counts < -2048))		// if ADC counts fall outside ADC valid range (this happens for saturation)
		{
			rtn = false;									// set return value to false, indicating measurement didn't take place correctly
		}
		else												// if ADC counts are in normal range
		{
			magX = mag_counts * Gauss_per_LSb + offsetx;	// calculate x-axis magnetic field, in Gauss
		}

		mag_counts = ( ((int)databuffer[4]) << 8 | databuffer[5] ) * MAG_YAXIS_SIGN;	// read y-axis adc counts
		if ((mag_counts > 2047) | (mag_counts < -2048))		// if ADC counts fall outside ADC valid range (this happens for saturation)
		{
			rtn = false;									// set return value to false, indicating measurement didn't take place correctly
		}
		else												// if ADC counts are in normal range
		{
			magY = mag_counts * Gauss_per_LSb + offsety;	// calculate y-axis magnetic field, in Gauss
		}

		mag_counts = ( ((int)databuffer[2]) << 8 | databuffer[3] ) * MAG_ZAXIS_SIGN;	// read z-axis adc counts
		if ((mag_counts > 2047) | (mag_counts < -2048))		// if ADC counts fall outside ADC valid range (this happens for saturation)
		{
			rtn = false;									// set return value to false, indicating measurement didn't take place correctly
		}
		else												// if ADC counts are in normal range
		{
			magZ = mag_counts * Gauss_per_LSb + offsetz;	// calculate y-axis magnetic field, in Gauss
		}

	}

	if (datacount > 0) datacount--;	// decrement data counter

	return rtn;
}

/*
 * Class:		HMC5883
 * Function:	Calc_Heading()
 * Scope:		public
 * Arguments:	_lAccum	roll	- roll angle of craft (about x axis), in radians.  Defined with positive value meaning roll right.
 * 				_lAccum pitch	- pitch angle of craft (about y axis), in radians.  Defined with positive value meaning pitch up.
 * Description:	This function calculates and returns the heading, in radians.  A heading of zero
 * 				indicates due north, while a heading of pi indicates due south.   The _lAccum data
 * 				type implies this number is times 2^24.
 */
_lAccum HMC5883::Calc_Heading(_lAccum roll, _lAccum pitch){					// calculates heading
	_lAccum Head_X;											// x-component of magnetic field, when compensated for by roll and pitch
	_lAccum Head_Y;											// y-component of magnetic field, when compensated for by roll and pitch
	_lAccum cos_roll;										// cosine of roll angle
	_lAccum sin_roll;										// sin of roll angle
	_lAccum cos_pitch;										// cosine of pitch angle
	_lAccum sin_pitch;										// sin of pitch angle
	_lAccum sin_roll_x_magY;								// sin of roll angle times y-axis magnetic measurement
	_lAccum cos_roll_x_magZ;								// cosine of roll angle times z-axis magnetic measurement

	sin_roll = lsincoslk(roll,&cos_roll);					// calculate sin and cos of roll angle
	sin_pitch = lsincoslk(pitch,&cos_pitch);				// calculate sin and cos of pitch angle
	sin_roll_x_magY = lmullk(sin_roll,magY);				// calculate sin of roll angle times y-axis magnetic measurement
	cos_roll_x_magZ = lmullk(cos_roll,magZ);				// calculate cosine of roll angle times z-axis magnetic measurement

	// Tilt compensated Magnetic field X component:
	Head_X = lmullk(magX,cos_pitch) + lmullk(sin_roll_x_magY,sin_pitch) + lmullk(cos_roll_x_magZ,sin_pitch);
	// Tilt compensated Magnetic field Y component:
	Head_Y = lmullk(magY,cos_roll) - lmullk(magZ,sin_roll);

	// Magnetic Heading
	if (Head_X >= 0)					// if x-axis component of heading is positive (heading north between 270 and 90 degrees)
	{
		heading = latan2lk(Head_X,-Head_Y) - magnetic_declination; // calculate heading, corrected for magnetic declination
	}
	else								// if x-axis component of heading is negative (heading south between 90 and 270 degrees)
	{
		heading = latan2lk(-Head_X,Head_Y) + PILK - magnetic_declination; // calculate heading, corrected for magnetic declination
	}

	if (heading < -PILK) heading += TWOPILK;			// angle normalization
	else if (heading > PILK) heading -= TWOPILK;	// angle normalization

	return heading;
}

/*
 * Class:		HMC5883
 * Function:	Read_Mag_Data_And_Calc_Heading()
 * Scope:		public
 * Arguments:	_lAccum	roll	- roll angle of craft (about x axis), in radians.  Defined with positive value meaning roll right.
 * 				_lAccum pitch	- pitch angle of craft (about y axis), in radians.  Defined with positive value meaning pitch up.
 * Description:	read magnetic data and calculates and returns the heading.  returns invalid value
 * 				of -128 radians if error occurred while reading data.
 */
_lAccum HMC5883::Read_Mag_Data_And_Calc_Heading(_lAccum roll, _lAccum pitch){// read magnetic data and calculates and returns the heading.  returns invalid value of -128 radians if error occurred while reading data.

	if (Read_Mag_Data())				// read magnetic data
	{
		return Calc_Heading(roll, pitch);			// if data read successfully, calculate and return the heading
	}
	else
	{
		return (_lAccum)MINUS128LK;					// if data read failed, return invalid heading
	}
}

/*
 * Class:		HMC5883
 * Function:	GetIdentity1()
 * Scope:		public
 * Arguments:	None
 * Description: returns first character of device identifier
 */
char HMC5883::GetIdentity1(void){
	return identity[0];
}

/*
 * Class:		HMC5883
 * Function:	GetIdentity2()
 * Scope:		public
 * Arguments:	None
 * Description: returns second character of device identifier
 */
char HMC5883::GetIdentity2(void){
	return identity[1];
}

/*
 * Class:		HMC5883
 * Function:	GetIdentity3()
 * Scope:		public
 * Arguments:	None
 * Description: returns third character of device identifier
 */
char HMC5883::GetIdentity3(void){
	return identity[2];
}

/*
 * Class:		HMC5883
 * Function:	GetMagX()
 * Scope:		public
 * Arguments:	None
 * Description: returns the magnetic reading in x-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
 */
_lAccum HMC5883::GetMagX(void){
	return magX;
}

/*
 * Class:		HMC5883
 * Function:	GetMagY()
 * Scope:		public
 * Arguments:	None
 * Description: returns the magnetic reading in y-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
 */
_lAccum HMC5883::GetMagY(void){
	return magY;
}

/*
 * Class:		HMC5883
 * Function:	GetMagZ()
 * Scope:		public
 * Arguments:	None
 * Description: returns the magnetic reading in z-axis, in Gauss.  The _lAccum data type implies this number is times 2^24.
 */
_lAccum HMC5883::GetMagZ(void){
	return magZ;
}

/*
 * Class:		HMC5883
 * Function:	GetHeading()
 * Scope:		public
 * Arguments:	None
 * Description: returns the heading, in radians.  A heading of zero indicates due north, while a heading of pi indicates
 * 				due south.   The _lAccum data type implies this number is times 2^24.
 */
_lAccum HMC5883::GetHeading(void){
	return heading;
}

/*
 * Class:		HMC5883
 * Function:	GetMagDeclination()
 * Scope:		public
 * Arguments:	None
 * Description: returns the magnetic declination, in radians.  This is the difference between magnetic and true north,
 * 				where a positive value indicates that magnetic north is east of true north.   The _lAccum data type
 * 				implies this number is times 2^24.
 */
_lAccum HMC5883::GetMagDeclination(void){
	return magnetic_declination;
}

/*
 * Class:		HMC5883
 * Function:	GetDataCount()
 * Scope:		public
 * Arguments:	None
 * Description: returns the number of unread data items awaiting in the HMC5883.
 */
byte HMC5883::GetDataCount(void){
	return datacount;
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

	Wire.beginTransmission(COMPASS_ADDRESS);			// Begin I2C transmission with the magnetometer
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

// Global Functions

/*
 * Class:		NA
 * Function:	HMC5883_data_int()
 * Scope:		global
 * Arguments:	none
 * Description:	HMC5883 INTERRUPT ON INT1 - calls function data_int() in class Hmc5883
 */
 void HMC5883_data_int(void)
 {
	 Hmc5883.data_int();
	 return;
 }
