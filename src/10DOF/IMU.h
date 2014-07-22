#ifndef IMU_H_
#define IMU_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"

#include "I2C.h"
#include "itg3200.h"
#include "adxl345.h"

class IMU {
public:
	ITG3200 gyro;
	ADXL345 accelerometer;
private:
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometr?
	inline void __attribute__((always_inline)) initialize() {
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

public:
	IMU(): gyro(), accelerometer(){
		initialize();
	}
	virtual ~IMU();
};

#endif
