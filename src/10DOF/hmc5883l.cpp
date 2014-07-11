#include "hmc5883l.h"
#include "IMU.h"

/** Default constructor, uses default I2C address.
 * @see HMC5883L_DEFAULT_ADDRESS
 */
HMC5883L::HMC5883L() {
    devAddr = HMC5883L_DEFAULT_ADDRESS;
}

/** Specific address constructor.
 * @param address I2C address
 * @see HMC5883L_DEFAULT_ADDRESS
 * @see HMC5883L_ADDRESS_AD0_LOW
 * @see HMC5883L_ADDRESS_AD0_HIGH
 */
HMC5883L::HMC5883L(uint8_t address) {
    devAddr = address;
}

/** Power on and prepare for general usage.
 * This will activate the gyroscope, so be sure to adjust the power settings
 * after you call this method if you want it to enter standby mode, or another
 * less demanding mode of operation. This also sets the gyroscope to use the
 * X-axis gyro for a clock source. Note that it doesn't have any delays in the
 * routine, which means you might want to add ~50ms to be safe if you happen
 * to need to read gyro data immediately after initialization. The data will
 * flow in either case, but the first reports may have higher error offsets.
 */
void HMC5883L::initialize() {
	if (testConnection()) {
		while (1) {
		}	//Fail_Handler();
	}
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
uint8_t HMC5883L::testConnection() {
//    return getDeviceID() == HMC5883L_I_AM;
}

void HMC5883L::test(NokiaLCD & nokia) {
		int16_t Out_x = 0, Out_y = 0, Out_z = 0;
		int32_t Sum_x = 0, Sum_y = 0, Sum_z = 0;

		for (uint16_t i = 0; i < 1000; i++) {
//			getRotation(&axis.x,&axis.y,&axis.z);
			Sum_x += axis.x;
			Sum_y += axis.y;
			Sum_z += axis.z;
		}
		Out_x = Sum_x / 1000;
		Out_y = Sum_y / 1000;
		Out_z = Sum_z / 1000;

		uint8_t buf[10];

		nokia.ClearLine(3);
//		sprintf((char*) buf, "X=%d", (int16_t) (Out_x * HMC5883L_2000G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 3);

		nokia.ClearLine(4);
//		sprintf((char*) buf, "Y=%d", (int16_t) (Out_y * HMC5883L_2000G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 4);

		nokia.ClearLine(5);
//		sprintf((char*) buf, "Z=%d", (int16_t) (Out_z * HMC5883L_2000G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 5);
	}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x34, 52 dec, 64 oct)
 * @see HMC5883L_RA_WHO_AM_I
 * @see HMC5883L_RA_DEVID_BIT
 * @see HMC5883L_RA_DEVID_LENGTH
 */
uint8_t HMC5883L::getDeviceID() {
//    IMU::i2c_ReadByte(devAddr, HMC5883L_RA_WHO_AM_I, buffer);
    return buffer[0];
}
/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see HMC5883L_RA_WHO_AM_I
 * @see HMC5883L_RA_DEVID_BIT
 * @see HMC5883L_RA_DEVID_LENGTH
 */
void HMC5883L::setDeviceID(uint8_t id) {
//    IMU::i2c_WriteBits(devAddr, HMC5883L_RA_WHO_AM_I, HMC5883L_DEVID_BIT, HMC5883L_DEVID_LENGTH, id);
}
