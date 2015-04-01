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
#include "SD/sdCardLogger.h"
#include "main.h"

// Global variable //
IMU imu10DOF;

void IMU::timerAction() {
	if(computationInProgress == 1) {
		error = 1;
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, "ComputationInProgress");
	}
	static uint8_t errorSensor = 0;
	uint8_t res[3];
	res[0] = accelerometer.update();
	res[1] = gyro.update();
	res[2] = magnetometer.update();

	if(res[0] || res[1] || res[2]) {
		if(errorSensor == 0) {
			errorSensor = 1;
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, "SensorUpdateError");
		}
	} else {
		errorSensor = 0;
		newRawDataAvailable = 1;
		doAllComputation();
		sprintf((char*) buf, "%d,%d,%d\n",(int16_t)(XRollAngle*1000),(int16_t)(YPitchAngle*1000),(int16_t)(ZYawAngle*1000));
		sdCardLogger.writeStringForIMU((char *) buf);
	}


	/* Save data to temporary buffer */
	if (numberOfGatheredSamples < numberOfSamplesToGather) {
//		measurements[numberOfGatheredSamples][0] = accelerometer.axis[0];
//		measurements[numberOfGatheredSamples][1] = accelerometer.axis[1];
//		measurements[numberOfGatheredSamples][2] = accelerometer.axis[2];
//		measurements[numberOfGatheredSamples][3] = gyro.axis[0];
//		measurements[numberOfGatheredSamples][4] = gyro.axis[1];
//		measurements[numberOfGatheredSamples][5] = gyro.axis[2];
//		measurements[numberOfGatheredSamples][6] = magnetometer.axis[0];
//		measurements[numberOfGatheredSamples][7] = magnetometer.axis[1];
//		measurements[numberOfGatheredSamples][8] = magnetometer.axis[2];
		++numberOfGatheredSamples;
	}
}

// ----- TIM_IRQHandler() ----------------------------------------------------
extern "C" void TIM3_IRQHandler(void) {
	if (__HAL_TIM_GET_ITSTATUS(&imu10DOF.TimHandle, TIM_IT_UPDATE ) != RESET) {
		semaphore_timerInterrupt = 1;
		imu10DOF.timerAction();
		semaphore_timerInterrupt = 0;
		__HAL_TIM_CLEAR_IT(&imu10DOF.TimHandle, TIM_IT_UPDATE);
	}
}

uint8_t IMU::sendRawDataViaVirtualCom() {
	const uint8_t frameSize = 2 * 3;
	int16_t temp;
	uint32_t timeDelta = 0;
	if (error == 1) {
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, "Overwritten!");
		error = 0;
		return 0;
	}

	if ((request == 1) && (connected == 1) && (newRawDataAvailable == 1)) {
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

		newRawDataAvailable = 0;
		request = 0;

		return 3 * frameSize;
	}
	return 0;
}

void IMU::sendMahonyViaVirtualCom() {
	uint16_t numberOfChars;
	numberOfChars = sprintf((char*) buf, "%d,%d,%d\n", (int16_t)(eulerAnglesInRadMahony[0]*1000),(int16_t)(eulerAnglesInRadMahony[1]*1000),(int16_t)(eulerAnglesInRadMahony[2]*1000));
	VCP_write((void *) buf, numberOfChars);
}

void IMU::sendAngleViaVirtualCom() {
	if ((request == 1) && (connected == 1)) {
		sprintf((char*) buf, "x:%d,y:%d,z:%d", (int16_t)(XRollAngle*1000),(int16_t)(YPitchAngle*1000),(int16_t)(ZYawAngle*1000));
		VCP_writeStringFrame(frameAddress_Pecet, frameType_DataRequest, buf);
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
		uint16_t it = 0;
		volatile int16_t *pTemp;
		for (it = 0; it < numberOfSamplesToGather; it++) {
//			pTemp = measurements[it];
//			numberOfChars = sprintf((char*) buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n", pTemp[0], pTemp[1], pTemp[2],
//					pTemp[3], pTemp[4], pTemp[5], pTemp[6], pTemp[7], pTemp[8]);
//			VCP_write((void *) buf, numberOfChars);
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
	newRawDataAvailable = 0;
	connected = 0;
	request = 0;
	error = 0;
	samplingFrequency = 400;
	numberOfGatheredSamples = 0;
	numberOfSamplesToGather = 0;

	x_IMU = 0;
	y_IMU = 0;
	z_IMU = 0;

	height_GPS = 0;
	lattitude_GPS = 0;
	longtitude_GPS = 0;

	TiltAngle = 0;
	ZYawAngle = 0;
	XRollAngle = 0;
	YPitchAngle = 0;
}

void IMU::initialize() {
	I2C::initialize();
	flagsComunicationInterface[f_interface_sensors] = f_configured;
	accelerometer.initialize();
	flagsHardware[f_device_accelerometer] = f_configured;
	flagsHardware[f_device_accelerometer] = f_deviceWorking;

	gyro.initialize();
	flagsHardware[f_device_gyroscope] = f_configured;
	flagsHardware[f_device_gyroscope] = f_deviceWorking;

	magnetometer.initialize();
	flagsHardware[f_device_magnetometer] = f_configured;
	flagsHardware[f_device_magnetometer] = f_deviceWorking;

	pressure.initialize();
	flagsHardware[f_device_pressure] = f_configured;
	flagsHardware[f_device_pressure] = f_deviceWorking;

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
	if(newRawDataAvailable) {
		computationInProgress = 1;

		kalmanStepAction();
		mahonyStepAction();
		computePitchRollTilt();
		computeYaw();

		newRawDataAvailable = 0;
		computationInProgress = 0;
	}
}

void IMU::updateGPSData(struct gpsData_t *_gpsdata) {
	this->lattitude_GPS = _gpsdata->lat;
	this->longtitude_GPS = _gpsdata->lon;
	this->height_GPS = _gpsdata->alt;
}

