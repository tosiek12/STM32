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

//	computeAngles();
//	kalmanStepAction();

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

void IMU::showMeasurment(NokiaLCD& nokiaLCD) {
	//		accelerometer.test(nokiaLCD);
	//		gyro.test(nokiaLCD);
	magnetometer.test(nokiaLCD, 0);
	pressure.test(nokiaLCD, 1);
}

void IMU::calibrateAllSensors() {
	//gyro.calibrate(false,100);
	accelerometer.calibrate(true, 100);
	//magnetometer.calibrate(false);
}

void IMU::calibrateGyroProcedure() {

	Delay::delay_ms(1);

}

IMU::IMU() :
		gyro(), accelerometer(), magnetometer(), pressure() {
	sendDataTriger = 0;
	connected = 0;
	request = 0;
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
	ZYawAngle = atan2f(arg1,arg2);
}

void IMU::kalmanStepAction() {
	const float64_t RAD_TO_DEG = 57.29577951f;
	float64_t dt_inSec = 0.001;
	float64_t gyroXrate = -((float64_t) ((gyro.axis[0])));
	gyroXangle += gyroXrate * dt_inSec; // Without any filter
	float64_t gyroYrate = ((float64_t) ((gyro.axis[1])));
	gyroYangle += gyroYrate * dt_inSec; // Without any filter

	// Complementary filter
	compAngleX = (0.93 * (compAngleX + gyroYrate * dt_inSec))
			+ (0.07 * XRollAngle);
	compAngleY = (0.93 * (compAngleY + gyroYrate * dt_inSec))
			+ (0.07 * YPitchAngle);
	// Kalman filter
	kalmanX.stepNewVersion(XRollAngle, gyroXrate, dt_inSec);
	kalmanY.stepNewVersion(YPitchAngle, gyroYrate, dt_inSec);
//	kalmanX.stepOldVersion(XRollAngle, gyroXrate, dt_inSec);
//	kalmanY.stepOldVersion(YPitchAngle, gyroYrate, dt_inSec);
}

void IMU::showAnglesKalman(NokiaLCD& nokiaLCD) {
	uint8_t buf[10];
	nokiaLCD.ClearLine(0);
	sprintf((char*) ((buf)), "X=%d", (int16_t) ((compAngleX)));
	nokiaLCD.WriteTextXY((char*) ((buf)), 0, 0);
	nokiaLCD.ClearLine(1);
	sprintf((char*) ((buf)), "Y=%d", (int16_t) ((compAngleY)));
	nokiaLCD.WriteTextXY((char*) ((buf)), 0, 1);
	nokiaLCD.ClearLine(2);
	sprintf((char*) ((buf)), "X_K=%d", (int16_t) ((kalmanX.getAngle())));
	nokiaLCD.WriteTextXY((char*) ((buf)), 0, 2);
	nokiaLCD.ClearLine(3);
	sprintf((char*) ((buf)), "Y_K=%d", (int16_t) ((kalmanY.getAngle())));
	nokiaLCD.WriteTextXY((char*) ((buf)), 0, 3);
}
