#ifndef IMU_H_
#define IMU_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"
#include "adxl345.h"
#include "itg3200.h"
#include "stdio.h"

#define TIMEOUT 4000

class IMU {
private:
	typedef struct {
		int16_t x;
		int16_t y;
		int16_t z;
	} LIS3DSH_OutXYZTypeDef;

	LIS3DSH_OutXYZTypeDef axis;
	//Sam sobie tlumaczy na odpowiednie read/write address
	const uint8_t I2C_ID_ITG3200 = 0xD0;	//Gyroscope - (0x68<<1)
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometr?
	const uint8_t I2C_ID_HMC5883L = 0x3C;	//Magnetometer?

	/**I2C1 GPIO Configuration
	 PB6     ------> I2C1_SCL
	 PB9     ------> I2C1_SDA
	 */
	GPIO_TypeDef * I2C_SCL_PORT = GPIOB;
	const uint16_t I2C_SCL_PIN = GPIO_PIN_6;

	GPIO_TypeDef * I2C_SDA_PORT = GPIOB;
	const uint16_t I2C_SDA_PIN = GPIO_PIN_9;
	inline void __attribute__((always_inline)) initialize() {
		I2C_MspInit();

		__I2C1_CLK_ENABLE();
		Delay::delay_ms(2);
		__I2C1_FORCE_RESET();
		__I2C1_RELEASE_RESET();
		Delay::delay_ms(2);
		__I2C1_CLK_ENABLE();
		Delay::delay_ms(2);

		hi2c.Instance = I2C1;
		hi2c.Init.ClockSpeed = 400000;
		hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c.Init.OwnAddress1 = 0x10;
		hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		hi2c.Init.OwnAddress2 = 0x10;
		hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
		HAL_I2C_Init(&hi2c);
		__HAL_I2C_ENABLE(&hi2c);
		Delay::delay_ms(2);

//		hmc5883l_initialize();
//		bmp085_initialize();
		adxl345_initialize();
//		itg3200_initialize();

	}

	uint8_t data_[10] = { 0 };
	/*
	 * Gyroscope check, and initialization.
	 */
	inline void __attribute__((always_inline)) itg3200_initialize() {
		volatile HAL_StatusTypeDef a = HAL_ERROR;

		if ((a = HAL_I2C_IsDeviceReady(&hi2c, I2C_ID_ITG3200, 10, TIMEOUT))
				!= HAL_OK) {
			while (1) {
			}	//Fail_Handler();
		}

		data_[0] = ITG3200_RA_WHO_AM_I;
		HAL_I2C_Master_Transmit(&hi2c, I2C_ID_ITG3200, (uint8_t*) data_, 1,
		TIMEOUT);

		a = HAL_I2C_Master_Receive(&hi2c, I2C_ID_ITG3200, (uint8_t*) data_, 1,
		TIMEOUT);
		if (data_[0] != ITG3200_DEFAULT_ADDRESS) {
			while (1) {
			}	//Fail_Handler();
		} else {
			//Initial setting setup.

		}
	}

	/*
	 * Compass check, and initialization.
	 */
	inline void __attribute__((always_inline)) hmc5883l_initialize() {
		volatile HAL_StatusTypeDef a = HAL_ERROR;

		if ((a = HAL_I2C_IsDeviceReady(&hi2c, I2C_ID_HMC5883L, 10, TIMEOUT))
				!= HAL_OK) {
			while (1) {
			}	//Fail_Handler();
		}

		data_[0] = ADXL345_RA_DEVID;
		HAL_I2C_Master_Transmit(&hi2c, I2C_ID_HMC5883L, (uint8_t*) data_, 1,
		TIMEOUT);

		a = HAL_I2C_Master_Receive(&hi2c, I2C_ID_HMC5883L, (uint8_t*) data_, 1,
		TIMEOUT);
		if (data_[0] != ADXL345_DEVID) {
			while (1) {
			}	//Fail_Handler();
		} else {
			//Initial setting setup.

		}
	}

	/*
	 * Pressure sensor check, and initialization.
	 */
	inline void __attribute__((always_inline)) bmp085_initialize() {
		volatile HAL_StatusTypeDef a = HAL_ERROR;

		if ((a = HAL_I2C_IsDeviceReady(&hi2c, I2C_ID_BMP085, 10, TIMEOUT))
				!= HAL_OK) {
			while (1) {
			}	//Fail_Handler();
		}

		data_[0] = ADXL345_RA_DEVID;
		HAL_I2C_Master_Transmit(&hi2c, I2C_ID_BMP085, (uint8_t*) data_, 1,
		TIMEOUT);

		a = HAL_I2C_Master_Receive(&hi2c, I2C_ID_BMP085, (uint8_t*) data_, 1,
		TIMEOUT);
		if (data_[0] != ADXL345_DEVID) {
			while (1) {
			}	//Fail_Handler();
		} else {
			//Initial setting setup.

		}
	}

	/*
	 * Accelerometer check, and initialization.
	 */
	inline void __attribute__((always_inline)) adxl345_initialize() {
		volatile HAL_StatusTypeDef a = HAL_ERROR;

		if ((a = HAL_I2C_IsDeviceReady(&hi2c, I2C_ID_ADXL345, 10, TIMEOUT))
				!= HAL_OK) {
			while (1) {
			}	//Fail_Handler();
		}

		data_[0] = ADXL345_RA_DEVID;
		HAL_I2C_Master_Transmit(&hi2c, I2C_ID_ADXL345, (uint8_t*) data_, 1,
		TIMEOUT);

		a = HAL_I2C_Master_Receive(&hi2c, I2C_ID_ADXL345, (uint8_t*) data_, 1,
		TIMEOUT);
		if (data_[0] != ADXL345_DEVID) {
			while (1) {
			}	//Fail_Handler();
		} else {

			//Need to set power control bit to wake up the adxl345
			data_[0] = ADXL345_DATA_RANGE_2G;
			HAL_I2C_Mem_Write(&hi2c, I2C_ID_ADXL345, ADXL345_RA_DATA_FORMAT,
			I2C_MEMADD_SIZE_8BIT, data_, 1, TIMEOUT);

			data_[0] = ADXL345_MEASURE_ENABLE;
			HAL_I2C_Mem_Write(&hi2c, I2C_ID_ADXL345, ADXL345_RA_POWER_CTL,
			I2C_MEMADD_SIZE_8BIT, data_, 1, TIMEOUT);	//Start Measurment

			data_[0] = ADXL345_BW_1600_HZ;
			HAL_I2C_Mem_Write(&hi2c, I2C_ID_ADXL345, ADXL345_RA_BW_RATE,
			I2C_MEMADD_SIZE_8BIT, data_, 1, TIMEOUT);	//Start Measurment
		}
	}

	inline void __attribute__((always_inline)) I2C_MspInit(void) {
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Peripheral clock enable */
		__GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

		GPIO_InitStruct.Pin = I2C_SCL_PIN;
		HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = I2C_SDA_PIN;
		HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
	}

public:
	IMU() {
		initialize();
	}
	virtual ~IMU();

	static I2C_HandleTypeDef hi2c;

	void test(NokiaLCD & nokia) {
		int16_t Out_x = 0, Out_y = 0, Out_z = 0;
		int32_t Sum_x = 0, Sum_y = 0, Sum_z = 0;

		for (uint16_t i = 0; i < 1000; i++) {
			IMU::i2c_ReadBuf(I2C_ID_ADXL345, ADXL345_RA_DATAX0, 6,
					(uint8_t *) &axis);
			Sum_x += axis.x;
			Sum_y += axis.y;
			Sum_z += axis.z;
		}
		Out_x = Sum_x / 1000;
		Out_y = Sum_y / 1000;
		Out_z = Sum_z / 1000;

		uint8_t buf[10];

		nokia.ClearLine(0);
		sprintf((char*) buf, "X=%d", (int16_t) (Out_x * ADXL345_2G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 0);

		nokia.ClearLine(1);
		sprintf((char*) buf, "Y=%d", (int16_t) (Out_y * ADXL345_2G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 1);

		nokia.ClearLine(2);
		sprintf((char*) buf, "Z=%d", (int16_t) (Out_z * ADXL345_2G_FACTOR));
		nokia.WriteTextXY((char*) buf, 0, 2);
	}

	static HAL_StatusTypeDef i2c_ReadByte(uint8_t devAddr, uint8_t regAddr,
			uint8_t* pBuf) {
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &pBuf, 1, TIMEOUT);
	}

	static HAL_StatusTypeDef i2c_WriteByte(uint8_t devAddr, uint8_t regAddr,
			uint8_t data) {
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &data, 1, TIMEOUT);
	}

	static HAL_StatusTypeDef i2c_ReadBuf(uint8_t devAddr, uint8_t regAddr,
			int16_t nBytes, uint8_t* pBuf) {
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	static HAL_StatusTypeDef i2c_ReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
		HAL_StatusTypeDef res = i2c_ReadByte(devAddr, regAddr, data);
	    *data = data[0] & (1 << bitNum);
	    return res;
	}

	static HAL_StatusTypeDef i2c_WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
	    uint8_t b;
	    HAL_StatusTypeDef res = i2c_ReadByte(devAddr, regAddr, &b);
	    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	    return i2c_WriteByte(devAddr, regAddr, b);
	}

	static HAL_StatusTypeDef i2c_WriteBuf(uint8_t devAddr, uint8_t regAddr,
			int16_t nBytes, uint8_t* pBuf) {
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	static HAL_StatusTypeDef i2c_WriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
	    //      010 value to write
	    // 76543210 bit numbers
	    //    xxx   args: bitStart=4, length=3
	    // 00011100 mask byte
	    // 10101111 original value (sample)
	    // 10100011 original & ~mask
	    // 10101011 masked | value
	    uint8_t b;
	    HAL_StatusTypeDef res;
	    if ((res = i2c_ReadByte(devAddr, regAddr, &b)) != HAL_OK) {
	        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	        data <<= (bitStart - length + 1); // shift data into correct position
	        data &= mask; // zero all non-important bits in data
	        b &= ~(mask); // zero all important bits in existing byte
	        b |= data; // combine data with existing byte
	        return i2c_WriteByte(devAddr, regAddr, b);
	    } else {
	        return res;
	    }
	}

	static HAL_StatusTypeDef i2c_ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
	    // 01101001 read byte
	    // 76543210 bit numbers
	    //    xxx   args: bitStart=4, length=3
	    //    010   masked
	    //   -> 010 shifted
		HAL_StatusTypeDef res;
	    uint8_t b;
	    if ((res = i2c_ReadByte(devAddr, regAddr, &b)) != HAL_OK) {
	        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	        b &= mask;
	        b >>= (bitStart - length + 1);
	        *data = b;
	    }
	    return res;
	}


};



#endif
