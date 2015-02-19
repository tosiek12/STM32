/*
 * IMU.cpp
 *
 *  Created on: 2 lip 2014
 *      Author: Antonio
 */
#include "10DOF/IMU.h"
#include "SpeedTester/speedTester.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}
#include "10DOF/Filters/MahonyAHRS.h"
#include "GPS/gps.h"
// Global variable //
IMU imu10DOF;

void IMU::timerAction() {
	if(computationInProgress == 1) {
		error = 1;
		VCP_write("$PCE", 4);
		VCP_write("OVF", 8);
		VCP_write("*", 1);
	}
	if (newDataAvailable == 1) {

		//return;
	}
	volatile uint8_t res[3];
	res[0] = accelerometer.update();
	res[1] = gyro.update();
	res[2] = magnetometer.update();

	GPS_SendCrucialData();

	if(res[0] || res[1] || res[2]) {
		VCP_write("$PCE", 4);
		VCP_write("Czujniki", 8);
		VCP_write("*", 1);
	}
	doAllComputation();


	/* Save data to temporary buffer */
	if (numberOfGatheredSamples < numberOfSamplesToGather) {
		measurements[numberOfGatheredSamples][0] = accelerometer.axis[0];
		measurements[numberOfGatheredSamples][1] = accelerometer.axis[1];
		measurements[numberOfGatheredSamples][2] = accelerometer.axis[2];
		measurements[numberOfGatheredSamples][3] = gyro.axis[0];
		measurements[numberOfGatheredSamples][4] = gyro.axis[1];
		measurements[numberOfGatheredSamples][5] = gyro.axis[2];
		measurements[numberOfGatheredSamples][6] = magnetometer.axis[0];
		measurements[numberOfGatheredSamples][7] = magnetometer.axis[1];
		measurements[numberOfGatheredSamples][8] = magnetometer.axis[2];
		++numberOfGatheredSamples;
	}
	newDataAvailable = 1;
}

// ----- TIM_IRQHandler() ----------------------------------------------------
extern "C" void TIM3_IRQHandler(void) {
	if (__HAL_TIM_GET_ITSTATUS(&imu10DOF.TimHandle, TIM_IT_UPDATE ) != RESET) {
		imu10DOF.timerAction();
		__HAL_TIM_CLEAR_IT(&imu10DOF.TimHandle, TIM_IT_UPDATE);
	}
}

uint8_t IMU::sendViaVirtualCom() {
	const uint8_t frameSize = 2 * 3;
	int16_t temp;
	uint32_t timeDelta = 0;
	if (error == 1) {
		//VCP_write("OVERWRITTEN!!", 13);
		error = 0;
		return 0;
	}

	if ((request == 1) && (connected == 1) && (newDataAvailable == 1)) {
		//VCP_write("D", 1);
		timeDelta = speedTester.delta();
		/*	Frame:
		 * [
		 *  timeDelta, 4BYTES
		 *  accelerometer, 3x2BYTES
		 *  gyroscope,	3x2BYTES
		 *  magnetometer,	3x2BYTES
		 *  heading,	2BYTES
		 *  ]
		 */
		VCP_write(&timeDelta, 4);
		VCP_write(accelerometer.axis, frameSize);
		VCP_write(gyro.axis, frameSize);
		VCP_write(magnetometer.axis, frameSize);
		temp = (int16_t) magnetometer.heading;
		VCP_write(&temp, 2);

		//VCP_write("\n", 1);

		newDataAvailable = 0;
		request = 0;

		return 3 * frameSize;
	}
	return 0;
}

void IMU::sendMahonyViaVirtualCom() {
	uint16_t size;
	uint8_t buff[50];
	size = sprintf((char*) buff, "%d,%d,%d\n", (int16_t)(eulerAnglesInRadMahony[0]*1000),(int16_t)(eulerAnglesInRadMahony[1]*1000),(int16_t)(eulerAnglesInRadMahony[2]*1000));
	VCP_write((void *) buff, size);
}

void IMU::sendAngleViaVirtualCom() {
	int16_t temp;
	uint32_t timeDelta = 0;
	if ((request == 1) && (connected == 1)) {
		uint16_t it = 0, size;
		uint8_t buff[50];
		size = sprintf((char*) buff, "x:%d,y:%d,z:%d", (int16_t)(XRollAngle*1000),(int16_t)(YPitchAngle*1000),(int16_t)(ZYawAngle*1000));

		VCP_write("$PCA", 4);
		VCP_write((void *) buff, size);
		VCP_write("*", 1);
		request = 0;
	}
}

void IMU::requestDataGathering(uint16_t timeToSampleInSec) {
	numberOfSamplesToGather = MIN(samplingFrequency*timeToSampleInSec, 6000);
	numberOfGatheredSamples = 0;
}

void IMU::sendGatheredDataViaVCOM() {
	if(numberOfSamplesToGather>0) {
		stopTimerUpdate();
		uint16_t it = 0, size;
		uint8_t buff[50];
		volatile int16_t *pTemp;
		for (it = 0; it < numberOfSamplesToGather; it++) {
			pTemp = measurements[it];
			size = sprintf((char*) buff, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", pTemp[0], pTemp[1], pTemp[2],
					pTemp[3], pTemp[4], pTemp[5], pTemp[6], pTemp[7], pTemp[8]);
			VCP_write((void *) buff, size);
			VCP_Flush();
		}
		numberOfGatheredSamples = 0;
		numberOfSamplesToGather = 0;
		startTimerUpdate();
	}
}

void IMU::calibrateGyroAndAccStationary() {
	stopTimerUpdate();
	accelerometer.calibrateStationary(100);
	gyro.calibrateStationary(100);
	startTimerUpdate();
}

IMU::IMU() :
		gyro(), accelerometer(), magnetometer(), pressure() {
	newDataAvailable = 0;
	connected = 0;
	request = 0;
	error = 0;

}

void IMU::initialize() {
	I2C::initialize();

//	accelerometer.initialize2();
//
//	while(1) {
//
//	}
	accelerometer.initialize();
	gyro.initialize();
	magnetometer.initialize();
	pressure.initialize();

	initializeTimerForUpdate();
}

void IMU::computeYaw() {
	float32_t sqrt_result, arg1, arg2;
	float32_t mag[3];
	mag[0] = magnetometer.axis_f[0];
	mag[1] = magnetometer.axis_f[1];
	mag[2] = magnetometer.axis_f[2];

	//Normalise measurement:
	arg1 = (float32_t) (mag[0]) * (mag[0]) + (mag[1]) * (mag[1]) + (mag[2]) * (mag[2]);
	arm_sqrt_f32(arg1, &sqrt_result);
	mag[0] /= sqrt_result;
	mag[1] /= sqrt_result;
	mag[2] /= sqrt_result;

	arg1 = arm_sin_f32(XRollAngle) * mag[2] - arm_cos_f32(XRollAngle) * mag[1];
	arg2 = arm_cos_f32(YPitchAngle) * mag[0]
			+ arm_sin_f32(XRollAngle) * arm_sin_f32(YPitchAngle) * mag[1]
			+ arm_cos_f32(XRollAngle) * arm_sin_f32(YPitchAngle) * mag[2];
	ZYawAngle = atan2f(arg1, arg2);

	//Correct declination, and change to compass angles
//	ZYawAngle += (5.0 + (16.0 / 60.0))*PI/180.0; //Positive declination.
//	if (ZYawAngle < 0) {
//		ZYawAngle += 2 * PI;
//	} else if (ZYawAngle > 2 * PI) {
//		ZYawAngle -= 2 * PI;
//	}
}

void IMU::computePitchRollTilt() {
	float32_t arg1;
	float32_t sqrt_result;
	float32_t acc[3];
	acc[0] = accelerometer.axis_f[0];
	acc[1] = accelerometer.axis_f[1];
	acc[2] = accelerometer.axis_f[2];

	//Normalise measurement:
	arg1 = (float32_t) (acc[0]) * (acc[0]) + (acc[1]) * (acc[1]) + (acc[2]) * (acc[2]);
	arm_sqrt_f32(arg1, &sqrt_result);
	acc[0] /= sqrt_result;
	acc[1] /= sqrt_result;
	acc[2] /= sqrt_result;
	int8_t signZ;
	const float32_t mi = 0.01;

	//Compute angles:
	//wzor jest nieprawdziwy, gdy z=0 i y=0. nie ma jednego dobreg rozwiazania. Mozna np. aproksymowac w ten sposob:
	if (acc[2] > 0) {
		signZ = 1;
	} else {
		signZ = -1;
	}
	arg1 = (acc[2]) * (acc[2]) + mi * (acc[0]) * (acc[0]);
	arm_sqrt_f32(arg1, &sqrt_result);
	XRollAngle = atan2(acc[1], signZ * sqrt_result);

	arg1 = 0;
	arg1 += (float32_t) (acc[2]) * (float32_t) (acc[2]);
	arg1 += (float32_t) (acc[1]) * (float32_t) (acc[1]);
	arm_sqrt_f32(arg1, &sqrt_result);
	YPitchAngle = atan2f(-(acc[0]), sqrt_result);
	//YPitchAngle_2 = arm_sin_f32((float32_t)(-acc[0])/(float32_t)(g));

	arg1 = 0;
	arg1 += (float32_t) (acc[0]) * (float32_t) (acc[0]);
	arg1 += (float32_t) (acc[1]) * (float32_t) (acc[1]);
	arg1 += (float32_t) (acc[2]) * (float32_t) (acc[2]);
	arm_sqrt_f32(arg1, &sqrt_result);
	TiltAngle = arm_cos_f32((acc[2]) / sqrt_result);
}

void IMU::mahonyStepAction() {
	MahonyAHRSupdate(gyro.axis_f[0],gyro.axis_f[1],gyro.axis_f[2],
			accelerometer.axis_f[0],accelerometer.axis_f[1],accelerometer.axis_f[2],
			magnetometer.axis_f[0],magnetometer.axis_f[1],magnetometer.axis_f[2]);
	MahonyToEuler(eulerAnglesInRadMahony);
}

void IMU::kalmanStepAction() {
	//const float64_t RAD_TO_DEG = 57.29577951f;
	float64_t dt_inSec = 0.0025;
	float64_t gyroXrate = -((float64_t) ((gyro.axis_f[0])));
	gyroXangle += gyroXrate * dt_inSec; // Without any filter
	float64_t gyroYrate = ((float64_t) ((gyro.axis_f[1])));
	gyroYangle += gyroYrate * dt_inSec; // Without any filter

	// Complementary filter
	compAngleX = (0.93 * (compAngleX + gyroYrate * dt_inSec)) + (0.07 * XRollAngle);
	compAngleY = (0.93 * (compAngleY + gyroYrate * dt_inSec)) + (0.07 * YPitchAngle);
	// Kalman filter
	kalmanX.stepOldVersion(XRollAngle, gyroXrate, dt_inSec);
	kalmanY.stepOldVersion(YPitchAngle, gyroYrate, dt_inSec);
}

void IMU::doAllComputation() {
	if(newDataAvailable) {
		computationInProgress = 1;

		kalmanStepAction();
		mahonyStepAction();
		computePitchRollTilt();
		computeYaw();

		newDataAvailable = 0;
		computationInProgress = 0;
	}
	if(error) {
		VCP_write("OVERFLOW!",10);
		error = 0;
	}

}
