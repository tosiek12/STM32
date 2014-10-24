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

/* Definition for TIMx clock resources */
#define TIMx                           TIM3
#define TIMx_CLK_ENABLE                __TIM3_CLK_ENABLE

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      (IRQn_Type) TIM3_IRQn
#define TIMx_IRQHandler                TIM3_IRQHandler

class IMU {
public:

private:
	ITG3200 gyro;
	ADXL345 accelerometer;
	HMC5883L magnetometer;
	BMP085 pressure;
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometr?
	volatile uint8_t sendDataTriger;
	volatile uint8_t showDataTriger;
	uint8_t connected;
	uint8_t request;

	Kalman kalmanX;
	Kalman kalmanY;
//	float64_t zeroValue[5] = { -200, 44, 660, 52.3, -18.5 }; // Found by experimenting
	/* All the angles start at 180 degrees */
	float64_t gyroXangle = 180;
	float64_t gyroYangle = 180;

	float64_t compAngleX = 180;
	float64_t compAngleY = 180;

	inline void __attribute__((always_inline)) initializeI2C() {
		I2C_HandleTypeDef hi2c;
		hi2c.Instance = I2C1;
		hi2c.Init.ClockSpeed = 400000;
		hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		hi2c.Init.OwnAddress1 = 0x10;
		hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		hi2c.Init.OwnAddress2 = 0x10;
		hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

		I2C::initialize(&hi2c);
	}

	inline void __attribute__((always_inline)) initializeTimerForUpdate() {
		/* TIMx Peripheral clock enable */
		__TIM3_CLK_ENABLE();

		const uint32_t CounterClk = 10000;	//Hz
		const uint16_t OutputClk = 1000;	//Hz
		//Prescaler = ((SystemCoreClock/2) / TIM3 counter clock) - 1
		const uint16_t Prescaler = (((SystemCoreClock / 2) / CounterClk) - 1);
		//ARR(TIM_Period) = (TIM3 counter clock / TIM3 output clock) - 1
		const uint32_t Period = ((CounterClk / OutputClk) - 1);

		/* Set TIMx instance */
		TimHandle.Instance = TIMx;
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
		HAL_NVIC_SetPriority(TIMx_IRQn, 4, 0);

		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIMx_IRQn);
		/* Start Channel1 */
		if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK) {
			/* Starting Error */
			while (1) {
			};
		}
	}

public:
	IMU() :
			gyro(), accelerometer(), magnetometer(), pressure() {
		sendDataTriger = 0;
		connected = 0;
		request = 0;
	}
	~IMU() {
	}
	TIM_HandleTypeDef TimHandle;
	void initialize();
	void showMeasurment(NokiaLCD &nokiaLCD) {
//		accelerometer.test(nokiaLCD);
//		gyro.test(nokiaLCD);
		magnetometer.test(nokiaLCD, 0);
		pressure.test(nokiaLCD, 1);
	}
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
	void computeAngles();

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

	void kalmanStepAction() {
		const float64_t RAD_TO_DEG = 57.29577951f;
		float64_t dt_inSec = 0.001;
		float64_t gyroXrate = -((float64_t) gyro.axis.x);
		gyroXangle += gyroXrate * dt_inSec; // Without any filter

		float64_t gyroYrate = ((float64_t) gyro.axis.y);
		gyroYangle += gyroYrate * dt_inSec; // Without any filter

		float64_t accXangle = (atan2(accelerometer.axis[0], accelerometer.axis[2]) + PI) * RAD_TO_DEG;
		float64_t accYangle = (atan2(accelerometer.axis[1], accelerometer.axis[2]) + PI) * RAD_TO_DEG;

		// Complementary filter
		compAngleX = (0.93 * (compAngleX + gyroYrate * dt_inSec))
				+ (0.07 * accXangle);
		compAngleY = (0.93 * (compAngleY + gyroYrate * dt_inSec))
				+ (0.07 * accYangle);

		// Kalman filter
		kalmanX.step(accXangle, gyroXrate, dt_inSec);
		kalmanY.step(accYangle, gyroYrate, dt_inSec);
	}
	void showAnglesKalman(NokiaLCD &nokiaLCD) {
		uint8_t buf[10];

		nokiaLCD.ClearLine(0);
		sprintf((char*) buf, "X=%d", (int16_t) compAngleX);
		nokiaLCD.WriteTextXY((char*) buf, 0, 0);

		nokiaLCD.ClearLine(1);
		sprintf((char*) buf, "Y=%d", (int16_t) compAngleY);
		nokiaLCD.WriteTextXY((char*) buf, 0, 1);

		nokiaLCD.ClearLine(2);
		sprintf((char*) buf, "X_K=%d", (int16_t) kalmanX.getAngle());
		nokiaLCD.WriteTextXY((char*) buf, 0, 2);

		nokiaLCD.ClearLine(3);
		sprintf((char*) buf, "Y_K=%d", (int16_t) kalmanY.getAngle());
		nokiaLCD.WriteTextXY((char*) buf, 0, 3);
	}
};

extern IMU imu10DOF;

#endif
