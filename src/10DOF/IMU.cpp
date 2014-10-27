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
	magnetometer.update();
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
		//imu10DOF.timerAction();
		__HAL_TIM_CLEAR_IT(&imu10DOF.TimHandle, TIM_IT_UPDATE);
	}

}

uint8_t IMU::sendViaVirtualCom() {
	const uint8_t frameSize = 6;
	uint16_t temp;
	if ((request == 1) && (connected == 1) && (sendDataTriger == 1)) {
		//VCP_write("D", 1);

		//Stop updateTimer()
		VCP_write(accelerometer.axis, frameSize);
		VCP_write(gyro.axis, frameSize);
		VCP_write(magnetometer.axis, frameSize);
		temp = (uint16_t) magnetometer.heading;
		VCP_write(&temp, 2);
		//Start updateTimer()

		//VCP_write("\n", 1);

		sendDataTriger = 0;
		request = 0;

		return 3 * frameSize;
	}
	return 0;
}

void IMU::calibrateAllSensors() {
	//gyro.calibrate(false,100);
	accelerometer.calibrate(true, 100);
	//magnetometer.calibrate(false);
}

void IMU::calibrateGyroProcedure() {

	Delay::delay_ms(1);

}

void IMU::initialize() {
	initializeI2C();
	gyro.initialize();
	accelerometer.initialize();
	magnetometer.initialize();
	pressure.initialize();

	//calibrateAllSensors();

	initializeTimerForUpdate();
}

void IMU::computeAngles() {
	float32_t XRollAngle;	//Range -180,180
	float32_t YPitchAngle;	//Range -90,90
	float32_t YPitchAngle_2;
	float32_t TiltAngle;		//Range 0,180	//odchylenie od pionu (grawitacji)
	float32_t Z_YawAngle;	//Range -180,180

	float32_t arg1, arg2;
	float32_t sqrt_result;
	const int16_t xActual= accelerometer.axis[0], yActual = accelerometer.axis[1], zActual = accelerometer.axis[2];
	const int16_t g = 435;	//TODO: uzupelnic wartosc g.

	const int16_t xMActual= magnetometer.axis[0], yMActual = magnetometer.axis[1], zMActual = magnetometer.axis[2];
	const int16_t MaxB = 123;	//TODO: uzupelnic wartos max do skalowania.


	XRollAngle = atan2f((float32_t)(yActual), (float32_t)(zActual))*180/PI;	//zgodne z teori¹

	arg1 = 0;
	arg1 +=(float32_t)(zActual)*(float32_t)(zActual);
	arg1 +=(float32_t)(yActual)*(float32_t)(yActual);
	arm_sqrt_f32(arg1,&sqrt_result);
	YPitchAngle = atan2f(-(xActual),sqrt_result)*180/PI;
	YPitchAngle_2 = asinf((float32_t)(-xActual)/(float32_t)(g))*180/PI;

	arg1 = 0;
	arg1 +=(float32_t)(zActual)*(float32_t)(zActual);
	arg1 +=(float32_t)(xActual)*(float32_t)(xActual);
	arg1 +=(float32_t)(yActual)*(float32_t)(yActual);
	arm_sqrt_f32(arg1,&sqrt_result);
	TiltAngle = acosf((zActual)/sqrt_result)*180/PI;

	arg1 = sinf(XRollAngle)*zMActual - cosf(XRollAngle)*yMActual;
	arg2 = cosf(YPitchAngle)*xMActual + sinf(XRollAngle)*sinf(YPitchAngle)* yMActual + cosf(XRollAngle)*sinf(YPitchAngle)*zMActual;
	Z_YawAngle = atan2f(arg1,arg2);
}
