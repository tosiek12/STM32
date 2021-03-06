// I2Cdev library collection - HMC5883L I2C device class
// Based on Honeywell HMC5883L datasheet, 10/2010 (Form #900405 Rev B)
// 6/12/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-12 - fixed swapped Y/Z axes
//     2011-08-22 - small Doxygen comment fixes
//     2011-07-31 - initial release

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

#include "10DOF/hmc5883l.h"
#include "10DOF/IMU.h"

uint8_t HMC5883L::selfTest(NokiaLCD &nokia) {
	/*
	 * X & Y Ases +-1.16gauss
	 * Z Axis +-1.08gauss
	 *
	 * X & Y & Z Axes (GN = 100) 510LSb
	 */
	I2C::i2c_WriteByte(devAddr, HMC5883L_RA_CONFIG_A,
			(HMC5883L_RATE_15 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1)
					| HMC5883L_BIAS_POSITIVE
							<< (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

	// write CONFIG_B register
	setGain(HMC5883L_GAIN_660);

	// write MODE register
	setMode(HMC5883L_MODE_SINGLE);

	// 720/706/688
	getHeading(&axis[0], &axis[1], &axis[2]);

	getHeading(&axis[0], &axis[1], &axis[2]);

	getHeading(&axis[0], &axis[1], &axis[2]);

	return 1;
}

void HMC5883L::loadCalibration() {
	offset[0] = 120;
	offset[1] = 118;
	offset[2] = 31;

	gain[0][0] = 0.9670;
	gain[0][1] =-0.0091;
	gain[0][2] = 0.0265;

	gain[1][0] =-0.0091;
	gain[1][1] = 0.9517;
	gain[1][2] = 0.0071;

	gain[2][0] = 0.0265;
	gain[2][1] = 0.0071;
	gain[2][2] = 1.0875;
}
void HMC5883L::calibrate(bool doFullCalibartion) {
	int16_t Max_x = 0, Max_y = 0, Max_z = 0;
	int16_t Min_x = 0, Min_y = 0, Min_z = 0;
	while (doFullCalibartion) {
		getHeading(&axis[0], &axis[1], &axis[2]);
		if (axis[0] > Max_x) {
			Max_x = axis[0];
		}
		if (axis[1] > Max_y) {
			Max_y = axis[1];
		}
		if (axis[2] > Max_z) {
			Max_z = axis[2];
		}

		if (axis[0] < Min_x) {
			Min_x = axis[0];
		}
		if (axis[1] < Min_y) {
			Min_y = axis[1];
		}
		if (axis[2] < Min_z) {
			Min_z = axis[2];
		}
	}
}

uint8_t HMC5883L::update() {
	if(updateRaw()) {
		return 1;
	}
	//Remove offset
	axis_f[0] = axis[0] + offset[0];
	axis_f[1] = axis[1] + offset[1];
	axis_f[2] = axis[2] + offset[2];

	//Change to mili Gauss [mG]
	axis_f[0] = axis_f[0] * gain[0][0] + axis_f[1] * gain[0][1] + axis_f[2] * gain[0][2];
	axis_f[1] = axis_f[0] * gain[1][0] + axis_f[1] * gain[1][1] + axis_f[2] * gain[1][2];
	axis_f[2] = axis_f[0] * gain[2][0] + axis_f[1] * gain[2][1] + axis_f[2] * gain[2][2];
	return 0;
}

uint8_t HMC5883L::updateRaw() {
	if(getHeading(&axis[0], &axis[1], &axis[2])) {
		return 1;
	}
	return 0;
}

/** Default constructor, uses default I2C address.
 * @see HMC5883L_DEFAULT_ADDRESS
 */
HMC5883L::HMC5883L() {
	devAddr = HMC5883L_DEFAULT_ADDRESS;
	offset[0] = 0;
	offset[1] = 0;
	offset[2] = 0;
	scalingFactor = 0;
	mode = 0;
	heading = 0;
}

/** Specific address constructor.
 * @param address I2C address
 * @see HMC5883L_DEFAULT_ADDRESS
 * @see HMC5883L_ADDRESS
 */
HMC5883L::HMC5883L(uint8_t address) :
		HMC5883L() {
	devAddr = address;
}

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode (very low power requirements). Default settings include 8-sample
 * averaging, 30 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for more information).
 */
void HMC5883L::initialize() {
	// write CONFIG_A register
	I2C::i2c_WriteByte(devAddr, HMC5883L_RA_CONFIG_A,
			(HMC5883L_AVERAGING_8 << (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1))
					| (HMC5883L_RATE_30 << (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1))
					| (HMC5883L_BIAS_NORMAL
							<< (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1)));

	// write CONFIG_B register
	setGain(HMC5883L_GAIN_1090);

	// write MODE register
	setMode(HMC5883L_MODE_CONTINUOUS);
	loadCalibration();
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool HMC5883L::testConnection() {
	if (I2C::i2c_ReadBuf(devAddr, HMC5883L_RA_ID_A, 3, buffer) == 3) {
		return (buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3');
	}
	return false;
}

// CONFIG_A register

/** Get number of samples averaged per measurement.
 * @return Current samples averaged per measurement (0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_AVERAGING_8
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
uint8_t HMC5883L::getSampleAveraging() {
	I2C::i2c_ReadBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT,
	HMC5883L_CRA_AVERAGE_LENGTH, buffer);
	return buffer[0];
}
/** Set number of samples averaged per measurement.
 * @param averaging New samples averaged per measurement setting(0-3 for 1/2/4/8 respectively)
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_AVERAGE_BIT
 * @see HMC5883L_CRA_AVERAGE_LENGTH
 */
void HMC5883L::setSampleAveraging(uint8_t averaging) {
	I2C::i2c_WriteBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_AVERAGE_BIT,
	HMC5883L_CRA_AVERAGE_LENGTH, averaging);
}
/** Get data output rate value.
 * The Table below shows all selectable output rates in continuous measurement
 * mode. All three channels shall be measured within a given output rate. Other
 * output rates with maximum rate of 160 Hz can be achieved by monitoring DRDY
 * interrupt pin in single measurement mode.
 *
 * Value | Typical Data Output Rate (Hz)
 * ------+------------------------------
 * 0     | 0.75
 * 1     | 1.5
 * 2     | 3
 * 3     | 7.5
 * 4     | 15 (Default)
 * 5     | 30
 * 6     | 75
 * 7     | Not used
 *
 * @return Current rate of data output to registers
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
uint8_t HMC5883L::getDataRate() {
	I2C::i2c_ReadBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT,
	HMC5883L_CRA_RATE_LENGTH, buffer);
	return buffer[0];
}
/** Set data output rate value.
 * @param rate Rate of data output to registers
 * @see getDataRate()
 * @see HMC5883L_RATE_15
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_RATE_BIT
 * @see HMC5883L_CRA_RATE_LENGTH
 */
void HMC5883L::setDataRate(uint8_t rate) {
	I2C::i2c_WriteBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_RATE_BIT,
	HMC5883L_CRA_RATE_LENGTH, rate);
}
/** Get measurement bias value.
 * @return Current bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
uint8_t HMC5883L::getMeasurementBias() {
	I2C::i2c_ReadBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT,
	HMC5883L_CRA_BIAS_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement bias value.
 * @param bias New bias value (0-2 for normal/positive/negative respectively)
 * @see HMC5883L_BIAS_NORMAL
 * @see HMC5883L_RA_CONFIG_A
 * @see HMC5883L_CRA_BIAS_BIT
 * @see HMC5883L_CRA_BIAS_LENGTH
 */
void HMC5883L::setMeasurementBias(uint8_t bias) {
	I2C::i2c_WriteBits(devAddr, HMC5883L_RA_CONFIG_A, HMC5883L_CRA_BIAS_BIT,
	HMC5883L_CRA_BIAS_LENGTH, bias);
}

// CONFIG_B register

/** Get magnetic field gain value.
 * The table below shows nominal gain settings. Use the "Gain" column to convert
 * counts to Gauss. Choose a lower gain value (higher GN#) when total field
 * strength causes overflow in one of the data output registers (saturation).
 * The data output range for all settings is 0xF800-0x07FF (-2048 - 2047).
 *
 * Value | Field Range | Gain (LSB/Gauss)
 * ------+-------------+-----------------
 * 0     | +/- 0.88 Ga | 1370
 * 1     | +/- 1.3 Ga  | 1090 (Default)
 * 2     | +/- 1.9 Ga  | 820
 * 3     | +/- 2.5 Ga  | 660
 * 4     | +/- 4.0 Ga  | 440
 * 5     | +/- 4.7 Ga  | 390
 * 6     | +/- 5.6 Ga  | 330
 * 7     | +/- 8.1 Ga  | 230
 *
 * @return Current magnetic field gain value
 * @see HMC5883L_GAIN_1090
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
uint8_t HMC5883L::getGain() {
	I2C::i2c_ReadBits(devAddr, HMC5883L_RA_CONFIG_B, HMC5883L_CRB_GAIN_BIT,
	HMC5883L_CRB_GAIN_LENGTH, buffer);
	return buffer[0];
}
/** Set magnetic field gain value.
 * @param gain New magnetic field gain value
 * @see getGain()
 * @see HMC5883L_RA_CONFIG_B
 * @see HMC5883L_CRB_GAIN_BIT
 * @see HMC5883L_CRB_GAIN_LENGTH
 */
void HMC5883L::setGain(uint8_t gain_) {
	// use this method to guarantee that bits 4-0 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	switch (gain_) {
	case HMC5883L_GAIN_1370:
		scalingFactor = 0.73;	//(1000.0/HMC5883L_COEF_GAIN_1370)
		break;
	case HMC5883L_GAIN_1090:
		scalingFactor = 0.92;
		break;
	case HMC5883L_GAIN_820:
		scalingFactor = 1.22;
		break;
	case HMC5883L_GAIN_660:
		scalingFactor = 1.52;
		break;
	case HMC5883L_GAIN_440:
		scalingFactor = 2.27;
		break;
	case HMC5883L_GAIN_390:
		scalingFactor = 2.56;
		break;
	case HMC5883L_GAIN_330:
		scalingFactor = 3.03;
		break;
	case HMC5883L_GAIN_220:
		scalingFactor = 4.35;
		break;
	}

	I2C::i2c_WriteByte(devAddr, HMC5883L_RA_CONFIG_B,
			gain_ << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1));
}

// MODE register

/** Get measurement mode.
 * In continuous-measurement mode, the device continuously performs measurements
 * and places the result in the data register. RDY goes high when new data is
 * placed in all three registers. After a power-on or a write to the mode or
 * configuration register, the first measurement set is available from all three
 * data output registers after a period of 2/fDO and subsequent measurements are
 * available at a frequency of fDO, where fDO is the frequency of data output.
 *
 * When single-measurement mode (default) is selected, device performs a single
 * measurement, sets RDY high and returned to idle mode. Mode register returns
 * to idle mode bit values. The measurement remains in the data output register
 * and RDY remains high until the data output register is read or another
 * measurement is performed.
 *
 * @return Current measurement mode
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
uint8_t HMC5883L::getMode() {
	I2C::i2c_ReadBits(devAddr, HMC5883L_RA_MODE, HMC5883L_MODEREG_BIT,
	HMC5883L_MODEREG_LENGTH, buffer);
	return buffer[0];
}
/** Set measurement mode.
 * @param newMode New measurement mode
 * @see getMode()
 * @see HMC5883L_MODE_CONTINUOUS
 * @see HMC5883L_MODE_SINGLE
 * @see HMC5883L_MODE_IDLE
 * @see HMC5883L_RA_MODE
 * @see HMC5883L_MODEREG_BIT
 * @see HMC5883L_MODEREG_LENGTH
 */
void HMC5883L::setMode(uint8_t newMode) {
	// use this method to guarantee that bits 7-2 are set to zero, which is a
	// requirement specified in the datasheet; it's actually more efficient than
	// using the I2Cdev.writeBits method
	I2C::i2c_WriteByte(devAddr, HMC5883L_RA_MODE,
			mode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	mode = newMode; // track to tell if we have to clear bit 7 after a read
}

// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
uint8_t HMC5883L::getHeading(int16_t *x, int16_t *y, int16_t *z) {
	if(I2C::i2c_ReadBuf(devAddr, HMC5883L_RA_DATAX_H, 6, buffer)!= HAL_OK) {
		return 1;
	}
	if (mode == HMC5883L_MODE_SINGLE)
		I2C::i2c_WriteByte(devAddr, HMC5883L_RA_MODE,
		HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	*x = (((int16_t) buffer[0]) << 8) | buffer[1];
	*y = (((int16_t) buffer[4]) << 8) | buffer[5];
	*z = (((int16_t) buffer[2]) << 8) | buffer[3];
	return 0;
//	if(*x == -4096) {
//		*x = 0;
//	}
//	if(*y == -4096) {
//		*y = 0;
//	}
//	if(*z == -4096) {
//		*z = 0;
//	}
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t HMC5883L::getHeadingX() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	I2C::i2c_ReadBuf(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE)
		I2C::i2c_WriteByte(devAddr, HMC5883L_RA_MODE,
		HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t) buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t HMC5883L::getHeadingY() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	I2C::i2c_ReadBuf(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE)
		I2C::i2c_WriteByte(devAddr, HMC5883L_RA_MODE,
		HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t) buffer[4]) << 8) | buffer[5];
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t HMC5883L::getHeadingZ() {
	// each axis read requires that ALL axis registers be read, even if only
	// one is used; this was not done ineffiently in the code by accident
	I2C::i2c_ReadBuf(devAddr, HMC5883L_RA_DATAX_H, 6, buffer);
	if (mode == HMC5883L_MODE_SINGLE)
		I2C::i2c_WriteByte(devAddr, HMC5883L_RA_MODE,
		HMC5883L_MODE_SINGLE << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1));
	return (((int16_t) buffer[2]) << 8) | buffer[3];
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode
 * changed, two, the mode is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
bool HMC5883L::getLockStatus() {
	I2C::i2c_ReadBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer);
	return buffer[0];
}
/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
bool HMC5883L::getReadyStatus() {
	I2C::i2c_ReadBit(devAddr, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer);
	return buffer[0];
}

// ID_* registers

/** Get identification byte A
 * @return ID_A byte (should be 01001000, ASCII value 'H')
 */
uint8_t HMC5883L::getIDA() {
	I2C::i2c_ReadByte(devAddr, HMC5883L_RA_ID_A, buffer);
	return buffer[0];
}
/** Get identification byte B
 * @return ID_A byte (should be 00110100, ASCII value '4')
 */
uint8_t HMC5883L::getIDB() {
	I2C::i2c_ReadByte(devAddr, HMC5883L_RA_ID_B, buffer);
	return buffer[0];
}
/** Get identification byte C
 * @return ID_A byte (should be 00110011, ASCII value '3')
 */
uint8_t HMC5883L::getIDC() {
	I2C::i2c_ReadByte(devAddr, HMC5883L_RA_ID_C, buffer);
	return buffer[0];
}
