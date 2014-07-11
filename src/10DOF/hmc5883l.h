#ifndef HMC5883L_H_
#define HMC5883L_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"
#include "IMU.h"

//const uint8_t I2C_ID_HMC5883L = 0x3C;	//Magnetometer?
#define HMC5883L_DEFAULT_ADDRESS		0x3C

/*	Rejestr DEVID ---------  ------------------------------------------------
*	Default value: x110100x
*	Read/Write register
*/


/* Magnetometer */
class HMC5883L {
    public:
		HMC5883L();
        HMC5883L(uint8_t address);

		struct OutXYZTypeDef {
			int16_t x;
			int16_t y;
			int16_t z;
		};
		OutXYZTypeDef axis;

        void initialize();
        uint8_t testConnection();
        void test(NokiaLCD & nokia);

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);

    private:
        uint8_t devAddr;
        uint8_t buffer[6];
};

#endif
