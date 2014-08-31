/*
 * IMU.cpp
 *
 *  Created on: 2 lip 2014
 *      Author: Antonio
 */
#include "IMU.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}

// Global variable //
IMU imu10DOF;

void IMU::timerAction() {
	static uint16_t counter = 0;
	accelerometer.update();
	gyro.update();
	kalmanStepAction();
	sendDataTriger = 1;
	if (++counter == 200) {	//update LCD after x ms.
		showDataTriger = 1;
		counter = 0;
	}

}

// ----- TIM_IRQHandler() ----------------------------------------------------
extern "C" void TIM3_IRQHandler(void) {
	if (__HAL_TIM_GET_ITSTATUS(&imu10DOF.TimHandle, TIM_IT_UPDATE ) != RESET) {
		imu10DOF.timerAction();
		__HAL_TIM_CLEAR_IT(&imu10DOF.TimHandle, TIM_IT_UPDATE);
	}

}

uint8_t IMU::sendViaVirtualCom() {
	const uint8_t frameSize = 6;

	if ((request == 1 ) && (connected == 1) && (sendDataTriger == 1)) {
		VCP_write("D", 1);
		VCP_write(&accelerometer.axis, frameSize);
		VCP_write(&gyro.axis, frameSize);
		VCP_write("\n", 1);

		sendDataTriger = 0;
		request = 0;

		return 2 * frameSize + 2;
	}
	return 0;
}


void IMU::calibrateAllSensors() {
	gyro.calibrate();
	accelerometer.calibrate();
	magnetometer.calibrate();
}
