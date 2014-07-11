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

#define TIMEOUT 4000

class I2C {
public:
	I2C();

	I2C_HandleTypeDef hi2c;

	HAL_StatusTypeDef i2c_ReadByte(uint8_t devAddr, uint8_t regAddr,
			uint8_t* pBuf) {
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &pBuf, 1, TIMEOUT);
	}

	HAL_StatusTypeDef i2c_WriteByte(uint8_t devAddr, uint8_t regAddr,
			uint8_t data) {
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) &data, 1, TIMEOUT);
	}

	HAL_StatusTypeDef i2c_ReadBuf(uint8_t devAddr, uint8_t regAddr,
			int16_t nBytes, uint8_t* pBuf) {
		return HAL_I2C_Mem_Read(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	HAL_StatusTypeDef i2c_ReadBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
		HAL_StatusTypeDef res = i2c_ReadByte(devAddr, regAddr, data);
	    *data = data[0] & (1 << bitNum);
	    return res;
	}

	HAL_StatusTypeDef i2c_WriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
	    uint8_t b;
	    HAL_StatusTypeDef res = i2c_ReadByte(devAddr, regAddr, &b);
	    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	    return i2c_WriteByte(devAddr, regAddr, b);
	}

	HAL_StatusTypeDef i2c_WriteBuf(uint8_t devAddr, uint8_t regAddr,
			int16_t nBytes, uint8_t* pBuf) {
		return HAL_I2C_Mem_Write(&hi2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT,
				(uint8_t*) pBuf, nBytes, TIMEOUT);
	}

	HAL_StatusTypeDef i2c_WriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
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

	HAL_StatusTypeDef i2c_ReadBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
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

#endif /* I2C_H_ */
