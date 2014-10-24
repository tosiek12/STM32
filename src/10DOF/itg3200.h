#ifndef ITG3200_H_
#define ITG3200_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"
#include "I2C.h"

#define ITG3200_ADDRESS_AD0_LOW     (0x68<<1) // address pin low (GND), default for SparkFun IMU Digital Combo board
#define ITG3200_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC), default for SparkFun ITG-3200 Breakout board
#define ITG3200_DEFAULT_ADDRESS     ITG3200_ADDRESS_AD0_LOW

/*	Rejestr DEVID --------- Register 0 – Who Am I ------------------------------------------------
*	Default value: x110100x
*	Read/Write register
*/
#define ITG3200_RA_WHO_AM_I         0x00
#define ITG3200_DEVID_BIT           6
#define ITG3200_DEVID_LENGTH        6
#define ITG3200_I_AM				0b110100	//52

/*	Rejestr SMPLRT_DIV --------- Register 21 – Sample Rate Divider ------------------------------------------------
*	Default value: 00000000
*	Read/Write register
*
*	F_sample = F_internal / (SMPLRT_DIV+1), where F_internal is either 1kHz or 8kHz
*/
#define ITG3200_RA_SMPLRT_DIV       0x15

/*	Rejestr DLPF_FS --------- Register 22 – DLPF, Full Scale ------------------------------------------------
*	Default value: 00000000
*	Read/Write register
*/
#define ITG3200_RA_DLPF_FS          0x16

#define ITG3200_DF_FS_SEL_BIT       4
#define ITG3200_DF_FS_SEL_LENGTH    2
#define ITG3200_FULLSCALE_2000      0x03
//Sensitivity from datasheet for 10bit resolution [mg/LSB]:
#define  ITG3200_2000G_FACTOR    ((float32_t)0.067565)     // 14.375 LSB/(grad/s)

#define ITG3200_DF_DLPF_CFG_BIT     2
#define ITG3200_DF_DLPF_CFG_LENGTH  3
#define ITG3200_DLPF_BW_256         0x00
#define ITG3200_DLPF_BW_188         0x01
#define ITG3200_DLPF_BW_98          0x02
#define ITG3200_DLPF_BW_42          0x03
#define ITG3200_DLPF_BW_20          0x04
#define ITG3200_DLPF_BW_10          0x05
#define ITG3200_DLPF_BW_5           0x06

/*	Rejestr INT_CFG --------- Register 23 – Interrupt Configuration ------------------------------------------------
*	Default value: 00000000
*	Read/Write register
*/
#define ITG3200_RA_INT_CFG          0x17
#define ITG3200_INTCFG_ACTL_BIT             7
#define ITG3200_INTCFG_OPEN_BIT             6
#define ITG3200_INTCFG_LATCH_INT_EN_BIT     5
#define ITG3200_INTCFG_INT_ANYRD_2CLEAR_BIT 4
#define ITG3200_INTCFG_ITG_RDY_EN_BIT       2
#define ITG3200_INTCFG_RAW_RDY_EN_BIT       0

#define ITG3200_INTMODE_ACTIVEHIGH  0x00
#define ITG3200_INTMODE_ACTIVELOW   0x01

#define ITG3200_INTDRV_PUSHPULL     0x00
#define ITG3200_INTDRV_OPENDRAIN    0x01

#define ITG3200_INTLATCH_50USPULSE  0x00
#define ITG3200_INTLATCH_WAITCLEAR  0x01

#define ITG3200_INTCLEAR_STATUSREAD 0x00
#define ITG3200_INTCLEAR_ANYREAD    0x01

/*	Rejestr INT_STATUS --------- Register 26 – Interrupt Status ------------------------------------------------
*	Default value: 00000000
*	Read register
*/
#define ITG3200_RA_INT_STATUS       0x1A
#define ITG3200_INTSTAT_ITG_RDY_BIT         2
#define ITG3200_INTSTAT_RAW_DATA_READY_BIT  0

/*	Rejestr  --------- Registers 27 to 34 – Sensor Registers ------------------------------------------------
*	Default value: 00000000
*	Read register
*	TEMP_OUT_H/L 16-bit temperature data (2’s complement format)
*	GYRO_XOUT_H/L 16-bit X gyro output data (2’s complement format)
*	GYRO_YOUT_H/L 16-bit Y gyro output data (2’s complement format)
*	GYRO_ZOUT_H/L 16-bit Y gyro output data (2’s complement format)
*
*/
#define ITG3200_RA_TEMP_OUT_H       0x1B
#define ITG3200_RA_TEMP_OUT_L       0x1C
#define ITG3200_RA_GYRO_XOUT_H      0x1D
#define ITG3200_RA_GYRO_XOUT_L      0x1E
#define ITG3200_RA_GYRO_YOUT_H      0x1F
#define ITG3200_RA_GYRO_YOUT_L      0x20
#define ITG3200_RA_GYRO_ZOUT_H      0x21
#define ITG3200_RA_GYRO_ZOUT_L      0x22

/*	Rejestr PWR_MGM --------- Register 62 – Power Management ------------------------------------------------
*	Default value: 00000000
*	Read/Write register
*/
#define ITG3200_RA_PWR_MGM          0x3E
#define ITG3200_PWR_H_RESET_BIT     7
#define ITG3200_PWR_SLEEP_BIT       6
#define ITG3200_PWR_STBY_XG_BIT     5
#define ITG3200_PWR_STBY_YG_BIT     4
#define ITG3200_PWR_STBY_ZG_BIT     3

#define ITG3200_PWR_CLK_SEL_BIT     2
#define ITG3200_PWR_CLK_SEL_LENGTH  3
#define ITG3200_CLOCK_INTERNAL      0x00
#define ITG3200_CLOCK_PLL_XGYRO     0x01
#define ITG3200_CLOCK_PLL_YGYRO     0x02
#define ITG3200_CLOCK_PLL_ZGYRO     0x03
#define ITG3200_CLOCK_PLL_EXT32K    0x04
#define ITG3200_CLOCK_PLL_EXT19M    0x05

class ITG3200 {
    public:
		/** Default constructor, uses default I2C address.
		 * @see ITG3200_DEFAULT_ADDRESS
		 */
		ITG3200(uint8_t address = ITG3200_DEFAULT_ADDRESS) {
			devAddr = address;
		}

		struct OutXYZTypeDef {
			int16_t x;
			int16_t y;
			int16_t z;
		};
		// Values in (grad/sek)
		OutXYZTypeDef axis;

        void initialize();
        uint8_t testConnection();
        void test(NokiaLCD & nokia);
        void update();
        void calibrate(bool doFullCalibartion, const uint16_t numberOfSamples);
        void getOversampledValueAndSendViaCOM(const uint8_t numberOfSamples);

        // WHO_AM_I register
        uint8_t getDeviceID();
        void setDeviceID(uint8_t id);

        // SMPLRT_DIV register
        uint8_t getRate();
        void setRate(uint8_t rate);

        // DLPF_FS register
        uint8_t getFullScaleRange();
        void setFullScaleRange(uint8_t range);
        uint8_t getDLPFBandwidth();
        void setDLPFBandwidth(uint8_t bandwidth);

        // INT_CFG register
        uint8_t getInterruptMode();
        void setInterruptMode(uint8_t mode);
        uint8_t getInterruptDrive();
        void setInterruptDrive(uint8_t drive);
        uint8_t getInterruptLatch();
        void setInterruptLatch(uint8_t latch);
        uint8_t getInterruptLatchClear();
        void setInterruptLatchClear(uint8_t clear);
        uint8_t getIntDeviceReadyEnabled();
        void setIntDeviceReadyEnabled(uint8_t enabled);
        uint8_t getIntDataReadyEnabled();
        void setIntDataReadyEnabled(uint8_t enabled);

        // INT_STATUS register
        uint8_t getIntDeviceReadyStatus();
        uint8_t getIntDataReadyStatus();

        // TEMP_OUT_* registers
        int16_t getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(int16_t* x, int16_t* y, int16_t* z);
        int16_t getRotationX();
        int16_t getRotationY();
        int16_t getRotationZ();

        // PWR_MGM register
        void reset();
        uint8_t getSleepEnabled();
        void setSleepEnabled(uint8_t enabled);
        uint8_t getStandbyXEnabled();
        void setStandbyXEnabled(uint8_t enabled);
        uint8_t getStandbyYEnabled();
        void setStandbyYEnabled(uint8_t enabled);
        uint8_t getStandbyZEnabled();
        void setStandbyZEnabled(uint8_t enabled);
        uint8_t getClockSource();
        void setClockSource(uint8_t source);

    private:
        OutXYZTypeDef offset;
        uint8_t devAddr;
        uint8_t buffer[6];
};

#endif /* ITG3200_H_ */
