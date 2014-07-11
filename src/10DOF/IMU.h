#ifndef IMU_H_
#define IMU_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"

#include "itg3200.h"
#include "I2C.h"
#include "adxl345.h"

class IMU {
public:
	ITG3200 gyro;
	ADXL345 accelerometer;
	I2C i2c;
private:
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometr?
	inline void __attribute__((always_inline)) initialize() {
		I2C_MspInit();

		__I2C1_CLK_ENABLE();
		Delay::delay_ms(2);
		__I2C1_FORCE_RESET();
		__I2C1_RELEASE_RESET();
		Delay::delay_ms(2);
		__I2C1_CLK_ENABLE();
		Delay::delay_ms(2);

		i2c.hi2c.Instance = I2C1;
		i2c.hi2c.Init.ClockSpeed = 400000;
		i2c.hi2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		i2c.hi2c.Init.OwnAddress1 = 0x10;
		i2c.hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		i2c.hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		i2c.hi2c.Init.OwnAddress2 = 0x10;
		i2c.hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		i2c.hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
		HAL_I2C_Init(&(i2c.hi2c));
		__HAL_I2C_ENABLE(&(i2c.hi2c));
		Delay::delay_ms(2);

	}
	inline void __attribute__((always_inline)) I2C_MspInit(void) {
		GPIO_InitTypeDef GPIO_InitStruct;
		/**I2C1 GPIO Configuration
		 PB6     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		GPIO_TypeDef * I2C_SCL_PORT = GPIOB;
		const uint16_t I2C_SCL_PIN = GPIO_PIN_6;

		GPIO_TypeDef * I2C_SDA_PORT = GPIOB;
		const uint16_t I2C_SDA_PIN = GPIO_PIN_9;

		/* Peripheral clock enable */
		__GPIOB_CLK_ENABLE();

		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;

		GPIO_InitStruct.Pin = I2C_SCL_PIN;
		HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = I2C_SDA_PIN;
		HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
	}

public:
	IMU(): gyro(&i2c), accelerometer(&i2c){
		initialize();
	}
	virtual ~IMU();
};

#endif
