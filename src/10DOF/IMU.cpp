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

/* Global variable */
IMU imu10DOF;

/* Member method implementation */
IMU::IMU(void) :
		gyro(), accelerometer(), magnetometer(), pressure() {
	newRawDataAvailable = 0;
	connected = 0;
	request = 0;
	error = 0;
	samplingFrequencyInHz = 0;
	samplingPeriodInS = .0;
	numberOfGatheredSamples = 0;
	numberOfSamplesToGather = 0;

	positionInGlobalFrame_IMU[0] = 0;
	positionInGlobalFrame_IMU[1] = 0;
	positionInGlobalFrame_IMU[2] = 0;

	height_GPS = 0;
	lattitude_GPS = 0;
	longtitude_GPS = 0;

	TiltAngleInRad = 0;
	ZYawAngleInRad = 0;
	XRollAngleInRad = 0;
	YPitchAngleInRad = 0;
}

void IMU::initializeTimerForUpdate(void) {
	/* TIMx Peripheral clock enable */
	__TIM3_CLK_ENABLE();

	const uint32_t CounterClk = 10000;	//Hz
	const uint16_t OutputClk = 400;	//Hz
	samplingFrequencyInHz = OutputClk;
	samplingPeriodInS = 1.0 / OutputClk;
	//Prescaler = ((SystemCoreClock/2) / TIM3 counter clock) - 1
	const uint16_t Prescaler = (((SystemCoreClock / 2) / CounterClk) - 1);
	//ARR(TIM_Period) = (TIM3 counter clock / TIM3 output clock) - 1
	const uint32_t Period = ((CounterClk / OutputClk) - 1);

	/* Set TIMx instance */
	TimHandle.Instance = TIM3;
	TimHandle.Init.Period = Period;
	TimHandle.Init.Prescaler = Prescaler;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
		/* Initialization Error */
		while (1) {
		};
	}

	/* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ((IRQn_Type) TIM3_IRQn);
	/* Set Interrupt Group Priority */
	HAL_NVIC_SetPriority((IRQn_Type) TIM3_IRQn, 1, 2);
	/* Start Channel1 */
	if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
		/* Starting Error */
		while (1) {
		};
	}

	__HAL_TIM_DISABLE(&TimHandle);
}

void IMU::initialize(void) {
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

void IMU::timerAction(void) {
	static uint8_t counterForloggingData = 0;
	if (semahpore_computationInProgress == 1) {
		error = 1;
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Error,
				"semahpore_computationInProgress");
	}
	static uint8_t errorSensor = 0;
	uint8_t res[3];
	res[0] = accelerometer.update();
	res[1] = gyro.update();
	res[2] = magnetometer.update();
	pressure.update();

	if((++counterForloggingData) == 40) {
		counterForloggingData = 0;
		logDataOnSD();
	}
	if (res[0] || res[1] || res[2]) {
		if (errorSensor == 0) {
			errorSensor = 1;
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, "SensorUpdateError");
		}
	} else {
		errorSensor = 0;
		newRawDataAvailable = 1;
		doAllComputation();
	}

	/* Save data to temporary buffer */
	if (numberOfGatheredSamples < numberOfSamplesToGather) {

		++numberOfGatheredSamples;
	}
}
void IMU::logDataOnSD(void) {
	//Sent elements must be in the same amount and order as defined in header.
	//For header check SdCardLogger class private field: headerOfIMUFile.
	//All element must be followed by comma and in the end must be new line char.

	/* x,y,z,	//Raw orientation
	 * x_c,y_c,z_c,	//cooked orientation
	 * altG,lonG,latG,dop,hdop,	//position from GPS
	 * altP,	//height from pressure
	 * \n
	 */
	snprintf((char*) buf, 200, "%ld,%ld,%ld,"
			"%ld,%ld,%ld,"
			"%u,%ld,%ld,%lu,%lu,"
			"%lu,\n",
			(int32_t) (XRollAngleInRad * 1000), (int32_t) (YPitchAngleInRad * 1000), (int32_t) (ZYawAngleInRad * 1000),
			(int32_t) (eulerAnglesInRadMahony[0] * 1000), (int32_t) (eulerAnglesInRadMahony[1] * 1000), (int32_t) (eulerAnglesInRadMahony[2] * 1000),
			gps.alt, gps.lon, gps.lat, gps.dop, gps.hdop,
			(uint32_t) (pressure.altitude * 1000)
			);
	sdCardLogger.writeStringForIMU((char *) buf);
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

inline void __attribute__((always_inline)) IMU::selfTests(NokiaLCD &nokiaLCD) {
	magnetometer.selfTest(nokiaLCD);
}
void IMU::startTimerUpdate(void) {
	__HAL_TIM_ENABLE(&TimHandle);
}
void IMU::stopTimerUpdate(void) {
	__HAL_TIM_DISABLE(&TimHandle);
}

void IMU::setConnected(void) {
	connected = 1;
	request = 0;
	pressure.test();
}
void IMU::setDisconnected(void) {
	connected = 0;
	request = 0;
}
void IMU::setRequestOfData(void) {
	request = 1;
}

void IMU::startDataGathering(void) {
	numberOfGatheredSamples = 0;
}
uint8_t IMU::isDataGatheringComplete(void) {
	return (numberOfGatheredSamples >= numberOfSamplesToGather);
}

void IMU::requestDataGathering(uint16_t timeToSampleInSec) {
	numberOfSamplesToGather = MIN(samplingFrequencyInHz * timeToSampleInSec, 6000);
	numberOfGatheredSamples = 0;
}

void IMU::calibrateGyroAndAccStationary(void) {
	stopTimerUpdate();
	accelerometer.calibrateStationary(100);
	gyro.calibrateStationary(100);
	startTimerUpdate();
}

void IMU::computeYaw(void) {
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

	arg1 = arm_sin_f32(XRollAngleInRad) * mag[2] - arm_cos_f32(XRollAngleInRad) * mag[1];
	arg2 = arm_cos_f32(YPitchAngleInRad) * mag[0]
			+ arm_sin_f32(XRollAngleInRad) * arm_sin_f32(YPitchAngleInRad) * mag[1]
			+ arm_cos_f32(XRollAngleInRad) * arm_sin_f32(YPitchAngleInRad) * mag[2];
	ZYawAngleInRad = atan2f(arg1, arg2);

	//Correct declination, and change to compass angles
//	ZYawAngleInRad += (5.0 + (16.0 / 60.0))*PI/180.0; //Positive declination.
//	if (ZYawAngleInRad < 0) {
//		ZYawAngleInRad += 2 * PI;
//	} else if (ZYawAngleInRad > 2 * PI) {
//		ZYawAngleInRad -= 2 * PI;
//	}
}

void IMU::computePitchRollTilt(void) {
	float32_t arg1;
	float32_t sqrt_result;
	float32_t acc[3];
	acc[0] = accelerometer.axisInMPerSsquared[0];
	acc[1] = accelerometer.axisInMPerSsquared[1];
	acc[2] = accelerometer.axisInMPerSsquared[2];

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
	XRollAngleInRad = atan2(acc[1], signZ * sqrt_result);

	arg1 = 0;
	arg1 += (float32_t) (acc[2]) * (float32_t) (acc[2]);
	arg1 += (float32_t) (acc[1]) * (float32_t) (acc[1]);
	arm_sqrt_f32(arg1, &sqrt_result);
	YPitchAngleInRad = atan2(-(acc[0]), sqrt_result);
	//YPitchAngleInRad_2 = arm_sin_f32((float32_t)(-acc[0])/(float32_t)(g));

	arg1 = 0;
	arg1 += (float32_t) (acc[0]) * (float32_t) (acc[0]);
	arg1 += (float32_t) (acc[1]) * (float32_t) (acc[1]);
	arg1 += (float32_t) (acc[2]) * (float32_t) (acc[2]);
	arm_sqrt_f32(arg1, &sqrt_result);
	TiltAngleInRad = arm_cos_f32((acc[2]) / sqrt_result);
}

void IMU::mahonyStepAction(void) {
	const float32_t * const pGyro = gyro.axisInRadPerS, * const pAcc = accelerometer.axisInMPerSsquared, * const pMag = magnetometer.axis_f;
	MahonyAHRSupdate(pGyro[0], pGyro[1], pGyro[2],
			pAcc[0], pAcc[1], pAcc[2],
			pMag[0], pMag[1], pMag[2]);
	MahonyToEuler(eulerAnglesInRadMahony);
}

void IMU::kalmanStepAction(void) {
	//const float64_t RAD_TO_DEG = 57.29577951f;
	float64_t dt_inSec = 0.0025;
	float64_t gyroXrate = -((float64_t) ((gyro.axisInRadPerS[0])));
	gyroXangle += gyroXrate * dt_inSec; // Without any filter
	float64_t gyroYrate = ((float64_t) ((gyro.axisInRadPerS[1])));
	gyroYangle += gyroYrate * dt_inSec; // Without any filter

	// Complementary filter
	compAngleX = (0.93 * (compAngleX + gyroYrate * dt_inSec)) + (0.07 * XRollAngleInRad);
	compAngleY = (0.93 * (compAngleY + gyroYrate * dt_inSec)) + (0.07 * YPitchAngleInRad);
	// Kalman filter
	kalmanX.stepOldVersion(XRollAngleInRad, gyroXrate, dt_inSec);
	kalmanY.stepOldVersion(YPitchAngleInRad, gyroYrate, dt_inSec);
}

void IMU::doAllComputation(void) {
	if (newRawDataAvailable) {
		semahpore_computationInProgress = 1;

		mahonyStepAction();
		computePitchRollTilt();
		computeYaw();
		kalmanStepAction();

		updatePositionFromIMU();

		newRawDataAvailable = 0;
		semahpore_computationInProgress = 0;
	}
}

void IMU::updateGPSData(const struct gpsData_t *_gpsdata) {
	this->lattitude_GPS = _gpsdata->lat;
	this->longtitude_GPS = _gpsdata->lon;
	this->height_GPS = _gpsdata->alt;
	memcpy(&gps, _gpsdata, sizeof(struct gpsData_t));
}

void IMU::prepareDataFrame(uint8_t * const pBuff, int16_t buffFreeSpace) {
	uint8_t tempBuff[50] = { '\0' };
	uint8_t numberOfcharsToAppend = 0;

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "x:%d,y:%d,z:%d,",
			(int16_t) (XRollAngleInRad * 1000), (int16_t) (YPitchAngleInRad * 1000),
			(int16_t) (ZYawAngleInRad * 1000));
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "x_c:%d,y_c:%d,z_c:%d,",
			(int16_t) (eulerAnglesInRadMahony[0] * 1000),
			(int16_t) (eulerAnglesInRadMahony[1] * 1000),
			(int16_t) (eulerAnglesInRadMahony[2] * 1000));
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "altG:%u,lonG:%ld,latG:%ld,",
			height_GPS, longtitude_GPS,
			lattitude_GPS);
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "dop:%lu,hdop:%lu,sats:%hu,", gps.dop,
			gps.hdop, gps.sats);
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "hhmmss:%c%ch%c%cm%c%cs,", gps.hhmmss[0],gps.hhmmss[1],gps.hhmmss[2],gps.hhmmss[3],gps.hhmmss[4],gps.hhmmss[5]);
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	numberOfcharsToAppend = snprintf((char *) tempBuff, 50, "altP:%lu,presP:%lu,",(uint32_t) (pressure.altitude * 10) , (uint32_t) (pressure.pressure) );
	buffFreeSpace -= numberOfcharsToAppend;
	strncat((char *) pBuff, (const char *) tempBuff, buffFreeSpace);

	if (buffFreeSpace < 0) {
		Error_Handler();
	}
}

void IMU::updatePositionFromIMU(void) {
	removeCentrifugalForceEffect();
	transformAccelerationToGlobalFrame();
	computeMovementAndAddToPossition();
}

void IMU::removeCentrifugalForceEffect(void) {
	float32_t arg1 = .0, sumOfSquares = .0;
	float32_t result = .0;
	volatile float32_t centrifugalAcceleration[3] = { .0 };
	float32_t rotationThresholdInRadPerSec = 5.0;
	//Physical dimensions on board in mm
	const float32_t dx = 0.015,	//15 mm
			dy = 0.01,		//20 mm
			dz = 0.003;	//3 mm
	float32_t *const pGyroAxis = gyro.axisInRadPerS, *const pAccAxis = accelerometer.axisInMPerSsquared;

	if (pGyroAxis[0] > rotationThresholdInRadPerSec) {
		centrifugalAcceleration[1] = pGyroAxis[0] * pGyroAxis[0] * dx;
	}
	if (pGyroAxis[1] > rotationThresholdInRadPerSec) {
		centrifugalAcceleration[0] = pGyroAxis[1] * pGyroAxis[1] * dy;
	}
	if (pGyroAxis[2] > rotationThresholdInRadPerSec) {
		arm_sqrt_f32(sumOfSquares, &result);
		centrifugalAcceleration[0] = pGyroAxis[2] * pGyroAxis[2] * result;
		centrifugalAcceleration[1] = pGyroAxis[2] * pGyroAxis[2] * result;

		sumOfSquares = dx * dx + dy * dy;
		arg1 = dy / sumOfSquares;
		centrifugalAcceleration[0] *= arm_sin_f32(arg1);
		arg1 = dx / sumOfSquares;
		centrifugalAcceleration[1] *= arm_cos_f32(arg1);
	}
	pAccAxis[0] -= centrifugalAcceleration[0];
	pAccAxis[1] -= centrifugalAcceleration[1];
	pAccAxis[2] -= centrifugalAcceleration[2];
}

void IMU::transformAccelerationToGlobalFrame(void) {
	float32_t *const pAccAxis = accelerometer.axisInMPerSsquared;
	float32_t norm = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
	float32_t invQuaterion[4] = { q0, -q1, -q2, -q3 };
	//inverse of quaterion
	if ((norm - 1.0) > 1.001 || (norm - 1.0) < 0.999) {
		invQuaterion[0] /= norm;
		invQuaterion[1] /= norm;
		invQuaterion[2] /= norm;
		invQuaterion[3] /= norm;
	}
	//rotation of acceleration vector by given quaterion.
	float32_t result = .0;
	result += 2 * (-invQuaterion[2] * invQuaterion[2] - invQuaterion[3] * invQuaterion[3] + 0.5)
			* pAccAxis[0];
	result += 2 * (invQuaterion[1] * invQuaterion[2] + invQuaterion[0] * invQuaterion[3])
			* pAccAxis[1];
	result += 2 * (-invQuaterion[0] * invQuaterion[2] + invQuaterion[1] * invQuaterion[3])
			* pAccAxis[2];
	accelerationInGlobalFrame[0] = result;

	result = .0;
	result += 2 * (invQuaterion[1] * invQuaterion[2] - invQuaterion[0] * invQuaterion[3])
			* pAccAxis[0];
	result += 2 * (-invQuaterion[1] * invQuaterion[1] - invQuaterion[3] * invQuaterion[3] + 0.5)
			* pAccAxis[1];
	result += 2 * (invQuaterion[0] * invQuaterion[1] + invQuaterion[2] * invQuaterion[3])
			* pAccAxis[2];
	accelerationInGlobalFrame[1] = result;

	result = .0;
	result += 2 * (invQuaterion[1] * invQuaterion[3] + invQuaterion[0] * invQuaterion[2])
			* pAccAxis[0];
	result += 2 * (-invQuaterion[0] * invQuaterion[1] + invQuaterion[2] * invQuaterion[3])
			* pAccAxis[1];
	result += 2 * (-invQuaterion[1] * invQuaterion[1] - invQuaterion[2] * invQuaterion[2] + 0.5)
			* pAccAxis[2];
	accelerationInGlobalFrame[2] = result;
}

void IMU::computeMovementAndAddToPossition(void) {
	volatile float32_t possitionChange[3] = { .0 };
	const float32_t maxVelocityInMPerS = 20;
	//remove gravitation (get only linear acceleration).
	accelerationInGlobalFrame[2] -= 9.8;

	//compute movement
	possitionChange[0] = velocityInGlobalFrame[0] * samplingPeriodInS
			+ accelerationInGlobalFrame[0] * samplingPeriodInS * samplingPeriodInS * 0.5;
	possitionChange[1] = velocityInGlobalFrame[1] * samplingPeriodInS
			+ accelerationInGlobalFrame[1] * samplingPeriodInS * samplingPeriodInS * 0.5;
	possitionChange[2] = velocityInGlobalFrame[2] * samplingPeriodInS
			+ accelerationInGlobalFrame[2] * samplingPeriodInS * samplingPeriodInS * 0.5;

	//update position
	positionInGlobalFrame_IMU[0] += possitionChange[0];
	positionInGlobalFrame_IMU[1] += possitionChange[1];
	positionInGlobalFrame_IMU[2] += possitionChange[2];

	//update velocity
	if(velocityInGlobalFrame[0]<maxVelocityInMPerS) {
		velocityInGlobalFrame[0] += accelerationInGlobalFrame[0] * samplingPeriodInS;
	}
	if(velocityInGlobalFrame[1]<maxVelocityInMPerS) {
		velocityInGlobalFrame[1] += accelerationInGlobalFrame[1] * samplingPeriodInS;
	}
	if(velocityInGlobalFrame[2]<maxVelocityInMPerS) {
		velocityInGlobalFrame[2] += accelerationInGlobalFrame[2] * samplingPeriodInS;
	}




}

void IMU::setPossitionFromGPSToIMU(void) {

}
