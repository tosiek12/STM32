/*
 * I2C.cpp
 *
 *  Created on: 11 lip 2014
 *      Author: Antonio
 */

#include "SDCard/SPI.h"

// Create static fields	//
uint8_t SPI::initialized = 0;
SPI_HandleTypeDef SPI::hspi1;

