#ifndef IMU_H_
#define IMU_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"

#include "I2C.h"
#include "itg3200.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "bmp085.h"
#include "Filters/Kalman.h"

class IMU {
public:

private:
/* Sensors */
	ITG3200 gyro;
	ADXL345 accelerometer;
	HMC5883L magnetometer;
	BMP085 pressure;
	uint16_t samplingFrequency;
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometr?
/* Status */
	volatile uint8_t sendDataTriger;
	volatile uint8_t showDataTriger;
	uint8_t connected;
	uint8_t request;
	uint8_t error;
/*	Filter */
	Kalman kalmanX;
	Kalman kalmanY;
	float32_t XRollAngle;	//Range -180,180
	float32_t YPitchAngle;	//Range -90,90
	float32_t TiltAngle;		//Range 0,180	//odchylenie od pionu (grawitacji)
	float32_t ZYawAngle;	//Range -180,180
//	float64_t zeroValue[5] = { -200, 44, 660, 52.3, -18.5 }; // Found by experimenting
	/* All the angles start at 180 degrees */
	float64_t gyroXangle = 180;
	float64_t gyroYangle = 180;
	float64_t compAngleX = 180;
	float64_t compAngleY = 180;

	/* Temporary buffers */
	volatile int16_t measurements[6000][9];
	volatile uint16_t numberOfGatheredSamples;
	uint16_t numberOfSamplesToGather;

	inline void __attribute__((always_inline)) initializeTimerForUpdate() {
		/* TIMx Peripheral clock enable */
		__TIM3_CLK_ENABLE();

		const uint32_t CounterClk = 10000;	//Hz
		const uint16_t OutputClk = 400;	//Hz
		samplingFrequency = OutputClk;
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

		/* Set Interrupt Group Priority */
		HAL_NVIC_SetPriority((IRQn_Type) TIM3_IRQn, 4, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ((IRQn_Type) TIM3_IRQn);
		/* Start Channel1 */
		if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
			/* Starting Error */
			while (1) {
			};
		}
		__HAL_TIM_DISABLE(&TimHandle);
	}

public:
	IMU();
	~IMU() {
	}
	TIM_HandleTypeDef TimHandle;
	void initialize();
	void showMeasurment(NokiaLCD& nokiaLCD);
	inline uint8_t __attribute__((always_inline))  getShowDataTriger() {
		return showDataTriger;
	}
	inline void __attribute__((always_inline)) clearShowDataTriger() {
		showDataTriger = 0;
	}
	inline void __attribute__((always_inline)) selfTests(NokiaLCD &nokiaLCD) {
		magnetometer.selfTest(nokiaLCD);
	}
	void calibrateAllSensors();
	void calibrateGyroProcedure();
	void timerAction();

	void computePitchRollTilt();
	void computeYaw();

	void startTimerUpdate() {
		__HAL_TIM_ENABLE(&TimHandle);
	}
	void stopTimerUpdate() {
		__HAL_TIM_DISABLE(&TimHandle);
	}
	uint8_t sendViaVirtualCom();
	void setConnected() {
		connected = 1;
		request = 0;
	}
	void setDisconnected() {
		connected = 0;
		request = 0;
	}
	void setRequestOfData() {
		request = 1;
	}

	void startDataGathering() {
		numberOfGatheredSamples = 0;
	}
	void sendGatheredDataViaVCOM();
	void requestDataGathering(uint16_t numberOfSamples);
	uint8_t isDataGatheringComplete() {
		return (numberOfGatheredSamples >= numberOfSamplesToGather);
	}

	void kalmanStepAction();
	void showAnglesKalman(NokiaLCD& nokiaLCD);
};

extern IMU imu10DOF;
#endif
