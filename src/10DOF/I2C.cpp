/*
 * I2C.cpp
 *
 *  Created on: 11 lip 2014
 *      Author: Antonio
 */

#include "10DOF/I2C.h"

// Create static fields	//
uint8_t I2C::initialized = 0;
I2C_HandleTypeDef I2C::hi2c;

extern "C" {void I2C1_EV_IRQHandler(void) {
	  HAL_NVIC_ClearPendingIRQ((IRQn_Type) I2C1_EV_IRQn);
	  HAL_I2C_EV_IRQHandler(&I2C::hi2c);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {

}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {

}


}
