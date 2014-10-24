#ifndef ADXL345_H_
#define ADXL345_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"
#include "I2C.h"

#define I2C_ID_ADXL345								 0xA6	//(0x53<<1)
#define ADXL345_ADDRESS_ALT_LOW 					 0xA6 // alt address pin low (GND)
#define ADXL345_ADDRESS_ALT_HIGH 					 (0x1D<<1) // alt address pin high (VCC)
#define ADXL345_DEFAULT_ADDRESS ADXL345_ADDRESS_ALT_LOW

/*	Rejestr DEVID --------- Device ID. ------------------------------------------------
 *	Default value: 11100101
 *	Read register
 */
#define ADXL345_RA_DEVID 							 0x00
#define ADXL345_DEVID								0xE5

/*	Rejestr THRESH_TAP --------- Tap threshold. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_THRESH_TAP 	0x1D

/*	Rejestr OFSX --------- X-axis offset. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_OFSX      0x1E

/*	Rejestr OFSY --------- Y-axis offset. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_OFSY                              0x1F

/*	Rejestr OFSZ --------- Z-axis offset. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_OFSZ                              0x20

/*	Rejestr DUR --------- Tap duration. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_DUR 								 0x21

/*	Rejestr LATENT --------- Tap latency. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_LATENT                            0x22

/*	Rejestr WINDOW --------- Tap window. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_WINDOW                            0x23

/*	Rejestr THRESH_ACT --------- Activity threshold. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_THRESH_ACT 						 0x24

/*	Rejestr THRESH_INACT --------- Inactivity threshold. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_THRESH_INACT 					 0x25

/*	Rejestr TIME_INACT --------- Inactivity time. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_TIME_INACT 						 0x26

/*	Rejestr ACT_INACT_CTL --------- Axis enable control for activity and inactivity detection. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_ACT_INACT_CTL 					 0x27

#define ADXL345_ACT_ACDC_ENABLE						 0x01<<7
#define ADXL345_ACTX_ENABLE						     0x01<<6
#define ADXL345_ACTY_ENABLE						     0x01<<5
#define ADXL345_ACTZ_ENABLE						     0x01<<4
#define ADXL345_INACT_ACDC_ENABLE					 0x01<<3
#define ADXL345_INACTX_ENABLE					     0x01<<2
#define ADXL345_INACTY_ENABLE					     0x01<<1
#define ADXL345_INACTZ_ENABLE					     0x01

#define ADXL345_ACT_ACDC_DISABLE				     0x00<<7
#define ADXL345_ACTX_DISABLE						 0x00<<6
#define ADXL345_ACTY_DISABLE						 0x00<<5
#define ADXL345_ACTZ_DISABLE						 0x00<<4
#define ADXL345_INACT_ACDC_DISABLE					 0x00<<3
#define ADXL345_INACTX_DISABLE					     0x00<<2
#define ADXL345_INACTY_DISABLE					     0x00<<1
#define ADXL345_INACTZ_DISABLE					     0x00

/*	Rejestr THRESH_FF --------- Free-fall threshold. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_THRESH_FF 						 0x28

/*	Rejestr TIME_FF --------- Free-fall time. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_TIME_FF 							 0x29

/*	Rejestr TAP_AXES --------- Axis control for tap/double tap. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_TAP_AXES 						 0x2A

#define ADXL345_TAPAXES_SUP_ENABLE						0x01<<3
#define ADXL345_TAPAXES_TAPX_ENABLE						0x01<<2
#define ADXL345_TAPAXES_TAPY_ENABLE						0x01<<1
#define ADXL345_TAPAXES_TAPZ_ENABLE						0x01

#define ADXL345_TAPAXES_SUP_DISABLE					    0x00<<3
#define ADXL345_TAPAXES_TAPX_DISABLE				    0x00<<2
#define ADXL345_TAPAXES_TAPY_DISABLE				    0x00<<1
#define ADXL345_TAPAXES_TAPZ_DISABLE				    0x00

/*	Rejestr ACT_TAP_STATUS --------- Source of tap/double tap. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_ACT_TAP_STATUS 					 0x2B

/*	Rejestr BW_RATE --------- Data rate and power mode control. ------------------------------------------------
 *	Default value: 00001010
 *	Read/Write register
 */
#define ADXL345_RA_BW_RATE 							 0x2C

#define ADXL345_LowPower								0x00<<4
#define ADXL345_NormalPower								0x01<<4

#define ADXL345_BW_1600_HZ									0x0F
#define ADXL345_BW_800_HZ 									0x0E
#define ADXL345_BW_400_HZ 									0x0D
#define ADXL345_BW_200_HZ 									0x0C
#define ADXL345_BW_100_HZ 									0x0B
#define ADXL345_BW_50_HZ  									0x0A
#define ADXL345_BW_25_HZ  									0x09
#define ADXL345_BW_12_5_HZ									0x08
#define ADXL345_BW_6_25_HZ									0x07
#define ADXL345_BW_3_13_HZ									0x06
#define ADXL345_BW_1_56_HZ									0x05
#define ADXL345_BW_0_78_HZ									0x04
#define ADXL345_BW_0_39_HZ									0x03
#define ADXL345_BW_0_20_HZ									0x02
#define ADXL345_BW_0_10_HZ									0x01
#define ADXL345_BW_0_05_HZ									0x00

/*	Rejestr POWER_CTL --------- Power-saving features control. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_POWER_CTL 						 0x2D

#define ADXL345_LINK_ENABLE								0x01<<5
#define ADXL345_LINK_DISABLE							0x00<<5
#define ADXL345_ASLEEP_ENABLE							0x01<<4
#define ADXL345_ASLEEP_DISABLE							0x00<<4
#define ADXL345_MEASURE_ENABLE							0x01<<3
#define ADXL345_MEASURE_DISABLE							0x00<<3
#define ADXL345_SLEEP_ENABLE							0x01<<2
#define ADXL345_SLEEP_DISABLE							0x00<<2
#define ADXL345_WAKE_8HZ								0x00
#define ADXL345_WAKE_4HZ								0x01
#define ADXL345_WAKE_2HZ								0x02
#define ADXL345_WAKE_1HZ								0x03

/*	Rejestr INT_ENABLE --------- Interrupt enable control. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_INT_ENABLE 						 0x2E

#define ADXL345_INT_DATARDY_ENABLE						0x01<<7
#define ADXL345_INT_DATARDY_DISABLE						0x00<<7
#define ADXL345_INT_SINGLETAP_ENABLE					0x01<<6
#define ADXL345_INT_SINGLETAP_DISABLE					0x00<<6
#define ADXL345_INT_DOUBLETAP_ENABLE					0x01<<5
#define ADXL345_INT_DOUBLTAP_DISABLE					0x00<<5
#define ADXL345_INT_ACTIVITY_ENABLE						0x01<<4
#define ADXL345_INT_ACTIVITY_DISABLE					0x00<<4
#define ADXL345_INT_INACTIVITY_ENABLE					0x01<<3
#define ADXL345_INT_INACTIVITY_DISABLE					0x00<<3
#define ADXL345_INT_FREEFALL_ENABLE						0x01<<2
#define ADXL345_INT_FREEFALL_DISABLE					0x00<<2
#define ADXL345_INT_WATERMARK_ENABLE					0x01<<1
#define ADXL345_INT_WATERMARK_DISABLE					0x00<<1
#define ADXL345_INT_OVERRUN_ENABLE						0x01
#define ADXL345_INT_OVERRUN_DISABLE						0x00

/*	Rejestr INT_MAP --------- Interrupt mapping control. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_INT_MAP 							 0x2F

/*	Rejestr INT_SOURCE --------- Source of interrupts. ------------------------------------------------
 *	Default value: 00000010
 *	Read register
 */
#define ADXL345_RA_INT_SOURCE 						 0x30

/*	Rejestr DATA_FORMAT --------- Data format control. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_DATA_FORMAT 						 0x31

#define ADXL345_DATA_SELFTEST_ENABLE					0x01<<7
#define ADXL345_DATA_SELFTEST_DISABLE					0x00<<7
#define ADXL345_DATA_SPI_3								0x01<<6
#define ADXL345_DATA_SPI_4								0x00<<6
#define ADXL345_DATA_INT_INVERT_ENABLE					0x01<<5
#define ADXL345_DATA_INT_INVERT_DISABLE					0x00<5
#define ADXL345_DATA_FULLRES_ENABLE						0x01<<3
#define ADXL345_DATA_FULLRES_DISABLE					0x00<<3
#define ADXL345_DATA_JUSTIFY_LEFT						0x01<<2
#define ADXL345_DATA_JUSTIFY_RIGHT						0x00<<2
#define ADXL345_DATA_RANGE_2G							0x00
#define ADXL345_DATA_RANGE_4G							0x01
#define ADXL345_DATA_RANGE_8G							0x02
#define ADXL345_DATA_RANGE_16G							0x03

//Sensitivity from datasheet for 10bit resolution [mg/LSB]:
#define  ADXL345_2G_FACTOR    ((float)3.9)     // 256 LSB/g
#define  ADXL345_4G_FACTOR    ((float)7.8)     // 128 LSB/g
#define  ADXL345_8G_FACTOR    ((float)15.6)     //64  LSB/g
#define  ADXL345_16G_FACTOR   ((float)31.2)    //32 LSB/g

/*	Rejestr DATAX0 --------- X-Axis Data 0. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAX0 							 0x32

/*	Rejestr DATAX1 --------- X-Axis Data 1. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAX1                            0x33

/*	Rejestr DATAY0 --------- Y-Axis Data 0. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAY0                            0x34

/*	Rejestr DATAY1 --------- Y-Axis Data 1. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAY1                            0x35

/*	Rejestr DATAZ0 --------- Z-Axis Data 0. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAZ0                            0x36

/*	Rejestr DATAZ1 --------- Z-Axis Data 1. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_DATAZ1                            0x37

/*	Rejestr FIFO_CTL --------- FIFO control. ------------------------------------------------
 *	Default value: 00000000
 *	Read/Write register
 */
#define ADXL345_RA_FIFO_CTL 						 0x38

#define ADXL345_FIFO_BYPASS								0x00<<7
#define ADXL345_FIFO_FIFO								0x01<<7
#define ADXL345_FIFO_STREAM								0x02<<7
#define ADXL345_FIFO_TRIGGER							0x03<<7
#define ADXL345_TRIG_INT1								0x00<<5
#define ADXL345_TRIG_INT2								0x01<<5
#define ADXL345_SAMPLE_WATERMARKENABLE					0x00
#define ADXL345_SAMPLE_WATERMARKDISABLE					0x01

/*	Rejestr FIFO_STATUS  --------- FIFO status. ------------------------------------------------
 *	Default value: 00000000
 *	Read register
 */
#define ADXL345_RA_FIFO_STATUS 						 0x39

class ADXL345 {
public:
	int16_t axis[3];
	ADXL345(uint8_t address = ADXL345_DEFAULT_ADDRESS) {
		devAddr = address;
		axis[1] = 0;
		axis[2] = 0;
		axis[3] = 0;
	}

	void initialize();
	uint8_t testConnection();
	void update();
	void calibrate(bool doFullCalibartion, const uint16_t numberOfSamples);

	// WHO_AM_I register
	uint8_t getDeviceID();
	void setDeviceID(uint8_t id);

	void ReadTapThresh(uint8_t *tap);
	void WriteTapThresh(uint8_t tap);

	void ReadXOffSet(int8_t *xoff);
	void ReadYOffSet(int8_t *yoff);
	void ReadZOffSet(int8_t *zoff);
	void ReadXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff);
	void WriteXOffSet(int8_t xoff);
	void WriteYOffSet(int8_t yoff);
	void WriteZOffSet(int8_t zoff);
	void WriteXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff);

	void ReadDur(uint8_t *dur);
	void WriteDur(uint8_t dur);

	void ReadLat(uint8_t *lat);
	void WriteLat(uint8_t lat);

	void ReadWindow(uint8_t *win);
	void WriteWindow(uint8_t win);

	void ReadAct(uint8_t *act);
	void WriteAct(uint8_t act);

	void ReadThreshInact(int8_t *inact);
	void WriteThreshInact(int8_t inact);

	void ReadTimeInact(uint8_t *timinact);
	void WriteTimeInact(uint8_t timinact);

	void WriteACDC(uint8_t ActEN, uint8_t ActX, uint8_t ActY, uint8_t ActZ,
			uint8_t InActEN, uint8_t InActX, uint8_t InActY, uint8_t InActZ);
	void ReadACDC(uint8_t *acdc);

	void ReadThreshFF(uint8_t *threshff);
	void WriteThreshFF(uint8_t threshff);

	void ReadTimeFF(uint8_t *timeff);
	void WriteTimeFF(uint8_t timeff);

	void ReadTapAxes(uint8_t *tap_axes);
	void WriteTapAxes(uint8_t sup, uint8_t tapx, uint8_t tapy, uint8_t tapz);

	void ReadActTapStatus(uint8_t *status);
	void ReadBWRate(uint8_t *bw);

	void WriteBWRate(uint8_t pwr, uint8_t bw);
	void ReadPWRCtl(uint8_t *pwrctl);

	void WritePWRCtl(uint8_t link, uint8_t autosleep, uint8_t measure,
			uint8_t sleep, uint8_t wake);

	void WriteINTEnable(uint8_t DataRDY, uint8_t singletap, uint8_t doubletap,
			uint8_t act, uint8_t inact, uint8_t ff, uint8_t watermrk,
			uint8_t overrun);
	void ReadINTEnable(uint8_t *inten);

	void ReadINTSource(uint8_t *intsource);

	void ReadDataFormat(uint8_t *data);
	void WriteDataFormat(uint8_t selftest, uint8_t spi, uint8_t intinv,
			uint8_t fullres, uint8_t justify, uint8_t range);

	void ReadXYZ(int16_t *xdata, int16_t *ydata, int16_t *zdata);

	void ReadFIFOCtl(uint8_t *fifo);
	void WriteFIFOCtl(uint8_t fifo, uint8_t trigger, uint8_t sample);

	void ReadFIFOStatus(uint8_t *fifost);
private:
	int16_t offset[3];
	float32_t scalingFactor[3];
	uint8_t devAddr;
	uint8_t buffer[6];
};

#endif /* ADXL345_H_ */
