/*
 * I2C.cpp
 *
 *  Created on: 11 lip 2014
 *      Author: Antonio
 */

#include "I2C.h"

// Create static fields	//
uint8_t I2C::initialized = 0;
I2C_HandleTypeDef I2C::hi2c;

