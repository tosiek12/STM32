/*
 * I2C.h
 *
 *  Created on: 11 lip 2014
 *      Author: Antonio
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "../Delay/delay.h"
#include "main.h"

#define TIMEOUT 400

#define  I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_6

#define I2C_SDA_PORT  GPIOB
#define I2C_SDA_PIN GPIO_PIN_9

class I2C {
private:
	static I2C_HandleTypeDef hi2c;
	static uint8_t initialized;
	static void resetBus() {
		GPIO_InitTypeDef GPIO_InitStruct;

		//Change to manual clk
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pin = I2C_SCL_PIN;
		HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Pin = I2C_SDA_PIN;
		HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

		while (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) != GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(I2C_SCL_PORT, I2C_SCL_PIN);
			Delay::delay_ms(1);
		}

		while (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) != GPIO_PIN_SET) {
			HAL_GPIO_TogglePin(I2C_SCL_PORT, I2C_SCL_PIN);
			Delay::delay_ms(1);
		}

		//Return to basic configuration
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

		GPIO_InitStruct.Pin = I2C_SCL_PIN;
		HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = I2C_SDA_PIN;
		HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
	}

	static void I2C_MspInit() {
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
	I2C() { };
	static inline void __attribute__((always_inline)) initialize( ) {

		I2C_MspInit();

		resetBus();

		__I2C1_CLK_ENABLE();

		__I2C1_FORCE_RESET();
		__I2C1_RELEASE_RESET();

		hi2c.Instance = I2C1;
		hi2c.Init.ClockSpeed = 400000;
		hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c.Init.OwnAddress1 = 0x10;
		hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		hi2c.Init.OwnAddress2 = 0x10;
		hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

		if(HAL_I2C_Init(&hi2c)!= HAL_OK) {
			Error_Handler();
		}
		Delay::delay_ms(2);
		initialized = 1;
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_ReadByte(
			uint8_t devAddr, uint8_t regAddr, uint8_t* pBuf) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &pBuf, 1, TIMEOUT);
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_WriteByte(
			uint8_t devAddr, uint8_t regAddr, uint8_t data) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &data, 1, TIMEOUT);
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_ReadBuf(
			uint8_t devAddr, uint8_t regAddr, int16_t nBytes, uint8_t* pBuf) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_ReadBit(
			uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		HAL_StatusTypeDef res = i2c_ReadByte(devAddr, regAddr, data);
		*data = data[0] & (1 << bitNum);
		return res;
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_WriteBit(
			uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		uint8_t b;
		i2c_ReadByte(devAddr, regAddr, &b);
		b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
		return i2c_WriteByte(devAddr, regAddr, b);
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_WriteBuf(
			uint8_t devAddr, uint8_t regAddr, int16_t nBytes, uint8_t* pBuf) {
		if (initialized == 0) {
			return HAL_ERROR;
		}
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_WriteBits(
			uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length,
			uint8_t data) {
		//      010 value to write
		// 76543210 bit numbers
		//    xxx   args: bitStart=4, length=3
		// 00011100 mask byte
		// 10101111 original value (sample)
		// 10100011 original & ~mask
		// 10101011 masked | value
		if (initialized == 0) {
			return HAL_ERROR;
		}
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

	static inline HAL_StatusTypeDef __attribute__((always_inline)) i2c_ReadBits(
			uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length,
			uint8_t *data) {
		// 01101001 read byte
		// 76543210 bit numbers
		//    xxx   args: bitStart=4, length=3
		//    010   masked
		//   -> 010 shifted
		if (initialized == 0) {
			return HAL_ERROR;
		}
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

#endif /* I2C_H_ */
