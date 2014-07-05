#include "adxl345.h"
#include "IMU.h"

/*
 * @brief: Test ADXL345 module address
 * @param[in]: none
 * @param[out]: 1 if successful, 0 if not
 */
uint8_t adxl345_test(void){
	uint8_t temp;
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_DEVID, &temp);
	return temp == ADXL345_DEVID ? 1 : 0;
}

/*
 * @ brief: Read threshold value for tap interrupts
 * @ param[in]: ptr to variable
 * @ param[out]: none
 */
void  adxl345_ReadTapThresh(uint8_t *tap){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_TAP, tap);
}
/*
 * @ brief: Write threshold value for tap interrupts
 * @ param[in]: Threshold value
 * @ param[out]: none
 */
void adxl345_WriteTapThresh(uint8_t tap){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_TAP, tap);
}

/*
 * @brief: Read X axis OffSet
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadXOffSet(int8_t *xoff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSX, (uint8_t *)xoff);
}

/*
 * @brief: Read Y axis offset
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadYOffSet(int8_t *yoff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSY, (uint8_t *)yoff);
}

/*
 * @brief: Read Z axis offset
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadZOffSet(int8_t *zoff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSZ, (uint8_t *)zoff);
}

/*
 * @brief: Read X, Y, Z axis offset
 * @param[in]: ptr to variables
 * @param[out]: none
 */
void adxl345_ReadXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSX, (uint8_t *)xoff);
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSY, (uint8_t *)yoff);
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_OFSZ, (uint8_t *)zoff);
}

/*
 * @brief: Set X axis OffSet
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteXOffSet(int8_t xoff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSX, xoff);
}

/*
 * @brief: Set Y axis OffSet
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteYOffSet(int8_t yoff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSY, yoff);
}

/*
 * @brief: Set  axis OffSet
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteZOffSet(int8_t zoff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSZ, zoff);
}

/*
 * @brief: Set X,Y,Z axis offset
 * @param[in]: X offset value , Y offset value, Z offset value
 * @param[out]: none
 */
void adxl_WriteXYZOffSet(int8_t *xoff, int8_t *yoff, int8_t *zoff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSX, *xoff);
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSY, *yoff);
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_OFSZ, *zoff);
}

/*
 * @brief: Read the maximum time an event must be above Tap threshold to qualify as a tap
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadDur(uint8_t *dur){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_DUR, dur);
}

/*
 * @brief: Write the maximum time an event must be above Tap threshold to qualify as a tap
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteDur(uint8_t dur){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_DUR, dur);
}

/*
 * @brief: Read the wait time from the detection of a tap event to the start of the window
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadLat(uint8_t *lat){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_LATENT, lat);
}

/*
 * @brief: Read the wait time from the detection of a tap event to the start of the window
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteLat(uint8_t lat){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_LATENT, lat);
}

/*
 * @brief: Read the amount of time after expiration of the latency time during which a second tap can begin
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadWindow(uint8_t *win){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_WINDOW, win);
}

/*
 * @brief: Read the amount of time after the expiration of the latency time during which a second time valid tap can begin
 * @param[in]: value
 * @param[out]: none
 */
void adxl345_WriteWindow(uint8_t win){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_WINDOW, win);
}

/*
 * @brief: Read the threshold value for detecting activity
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadAct(uint8_t *act){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_ACT, act);
}

/*
 * @brief: Write the threshold value for detecting activity
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteAct(uint8_t act){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_ACT, act);
}

/*
 * @brief: Read the threshold value for detecting inactivity
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadThreshInact(int8_t *inact){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_INACT, (uint8_t *) inact);
}

/*
 * @brief: Write the threshold value for detecting inactivity
 * @param[in]: value
 * @param[out]: none
 */
void adxl345_WriteThreshInact(int8_t inact){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_INACT, inact);
}

/*
 * @brief: Read the amount of time that acceleration must be less than, for inactivity to be declared
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadTimeInact(uint8_t *timinact){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_TIME_INACT, timinact);
}

/*
 * @brief: Write the amount of time that acceleration must be less than, for inactivity to be declared
 * @param[in]: Value
 * @param[out]: none
 */
void adxl345_WriteTimeInact(uint8_t timinact){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_TIME_INACT, timinact);
}

/*
 * @brief: Control AC/DC coupled operation
 * @param[in]: Enable AC coupled operation for Activity
 * 					ADXL345_ACT_ACDC_ENABLE, ADXL345_ACT_ACDC_DISABLE
 * 			   Enable X axis for activity
 * 			   		ADXL345_ACTX_ENABLE, ADXL345_ACTX_DISABLE
 * 			   Enable Y axis for activity
 * 			   		ADXL345_ACTY_ENABLE, ADXL345_ACTY_DISABLE
 * 			   Enable Z axis for activity
 * 			   		ADXL345_ACTZ_ENABLE, ADXL345_ACTZ_DISABLE
 * 			   Enable AC coupled operation for Inactivity
 * 					ADXL345_INACT_ACDC_ENABLE, ADXL345_INACT_ACDC_DISABLE
 * 			   Enable X axis for inactivity
 * 			   		ADXL345_INACTX_ENABLE, ADXL345_INACTX_DISABLE
 * 			   Enable Y axis for inactivity
 * 			   		ADXL345_INACTY_ENABLE, ADXL345_INACTY_DISABLE
 * 			   Enable Z axis for inactivity
 * 			   		ADXL345_INACTZ_ENABLE, ADXL345_INACTZ_DISABLE
 * @param[out]: none
 */
void adxl345_WriteACDC(uint8_t ActEN, uint8_t ActX, uint8_t ActY, uint8_t ActZ, uint8_t InActEN, uint8_t InActX, uint8_t InActY, uint8_t InActZ){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_ACT_INACT_CTL, ActEN|ActX|ActY|ActZ|InActEN|InActX|InActY|InActZ);
}

/*
 * @brief: Read the configuration of the Activity Inactivity configuration register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadACDC(uint8_t *acdc){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_ACT_INACT_CTL,acdc);
}

/*
 * @brief: Read the FreeFall threshold value
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadThreshFF(uint8_t *threshff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_FF, threshff);
}

/*
 * @brief: Write the FreeFall threshold value
 * @param[in]: value
 * @param[out]: none
 */
void adxl345_WriteThreshFF(uint8_t threshff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_THRESH_FF, threshff);
}

/*
 * @brief: Read the FreeFall threshold time
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadTimeFF(uint8_t *timeff){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_TIME_FF,timeff);
}

/*
 * @brief: Write the FreeFall time threshold
 * @param[in]: value
 * @param[out]: none
 */
void adxl345_WriteTimeFF(uint8_t timeff){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_TIME_FF, timeff);
}


/*
 * @brief: Read the suppress, tap x/y/z data
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadTapAxes(uint8_t *tap_axes){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_TAP_AXES, tap_axes);
}

/*
 * @brief: Write to TAP_AXES register
 * @param[in]: Suppres bit
 * 				ADXL345_TAPAXES_SUP_ENABLE, ADXL345_TAPAXES_SUP_DISABLE
 * 			   TapX bit
 * 			   	ADXL345_TAPAXES_TAPX_ENABLE, ADXL345_TAPAXES_TAPX_DISABLE
 * 			   TapY bit
 * 			   	ADXL345_TAPAXES_TAPY_ENABLE, ADXL345_TAPAXES_TAPY_DISABLE
 * 			   TapZ bit
 * 			   	ADXL345_TAPAXES_TAPZ_ENABLE, ADXL345_TAPAXES_TAPZ_DISABLE
 *
 */
void adxl345_WriteTapAxes(uint8_t sup, uint8_t tapx, uint8_t tapy, uint8_t tapz){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_TAP_AXES , sup|tapx|tapy|tapz);
}

/*
 * @brief: Read Act/Tap status
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadActTapStatus(uint8_t *status){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_ACT_TAP_STATUS, status);
}

/*
 * @brief: Read the BW rate
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadBWRate(uint8_t *bw){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_BW_RATE,bw);
}

/*
 * @brief: Set power mode and bandwidth
 * @param[in]: Power mode
 * 				ADXL345_LowPower , ADXL345_NormalPower
 * 			   BW Rate
 * 			   	ADXL345_BW_1600
 * 			   	ADXL345_BW_800
 * 			   	ADXL345_BW_400
 * 			   	ADXL345_BW_200
 * 			   	ADXL345_BW_100
 * 			   	ADXL345_BW_50
 * 			   	ADXL345_BW_25
 * 			   	ADXL345_BW_12P5
 * 			   	ADXL345_BW_6P25
 * 			   	ADXL345_BW_3P13
 * 			   	ADXL345_BW_1P56
 * 			   	ADXL345_BW_0P78
 * 			   	ADXL345_BW_0P39
 * 			   	ADXL345_BW_0P20
 * 			   	ADXL345_BW_0P10
 * 			   	ADXL345_BW_0P05
 * @param[out]: none
 */
void adxl345_WriteBWRate(uint8_t pwr, uint8_t bw){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_BW_RATE, pwr|bw);
}

/*
 * @brief: Read the power ctl register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadPWRCtl(uint8_t *pwrctl){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_POWER_CTL, pwrctl);
}

/*
 * @brief: Write to the power ctl register
 * @param[in]: Link
 * 				ADXL345_LINK_ENABLE, ADXL345_LINK_DISABLE
 * 			   Auto Sleep
 * 			   	ADXL345_ASLEEP_ENABLE, ADXL345_ASLEEP_DISABLE
 * 			   Measure
 * 			   	ADXL345_MEASURE_ENABLE, ADXL345_MEASURE_DISABLE
 * 			   Sleep
 * 			   	ADXL345_SLEEP_ENABLE, ADXL345_SLEEP_DISABLE
 * 			   Wakeup
 * 			   	ADXL345_WAKE_8HZ
 * 			   	ADXL345_WAKE_4HZ
 * 			   	ADXL345_WAKE_2HZ
 * 			   	ADXL345_WAKE_1HZ;
 * @param[out]: none
 */
void adxl345_WritePWRCtl(uint8_t link,uint8_t autosleep, uint8_t measure, uint8_t sleep, uint8_t wake){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_POWER_CTL, link|autosleep|measure|sleep|wake);
//	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_POWER_CTL, 0x00<<5|0x01<<4|0x01<<3|0x01<<2|0x00);

}

/*
 * @brief: Enable interrupts
 * @param[in]: Data Ready
 * 				ADXL345_INT_DATARDY_ENABLE, ADXL345_INT_DATARDY_DISABLE
 * 			   Single Tap
 * 			   	ADXL345_INT_SINGLETAP_ENABLE, ADXL345_INT_SINGLETAP_DISABLE
 * 			   Double Tap
 * 			   	ADXL345_INT_DOUBLETAP_ENABLE, ADXL345_INT_DOUBLTAP_DISABLE
 * 			   Activity
 * 			   	ADXL345_INT_ACTIVITY_ENABLE, ADXL345_INT_ACTIVITY_DISABLE
 * 			   Inactivity
 * 			   	ADXL345_INT_INACTIVITY_ENABLE, ADXL345_INT_INACTIVITY_DISABLE
 * 			   Freefall
 * 			   	ADXL345_INT_FREEFALL_ENABLE, ADXL345_INT_FREEFALL_DISABLE
 * 			   Water Mark
 * 			   	ADXL345_INT_WATERMARK_ENABLE, ADXL345_INT_WATERMARK_DISABLE
 * 			   Over Run
 * 			   	ADXL345_INT_OVERRUN_ENABLE, ADXL345_INT_OVERRUN_DISABLE
 * @param[out]: none
 */
void adxl345_WriteINTEnable(uint8_t DataRDY, uint8_t singletap, uint8_t doubletap, uint8_t act, uint8_t inact, uint8_t ff, uint8_t watermrk, uint8_t overrun){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_INT_ENABLE, DataRDY|singletap|doubletap|act|inact|ff|watermrk|overrun);
}

/*
 * @brief: Read from the enabled interrupts
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadINTEnable(uint8_t *inten){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_INT_ENABLE,inten);
}

/*No functions for INT_MAP register*/

/*
 * @brief: Read from the int source register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadINTSource(uint8_t *intsource){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_INT_SOURCE,intsource);
}

/*
 * @brief: Read from the data format register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadDataFormat(uint8_t *data){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_DATA_FORMAT,data);
}

/*
 * @brief: Write to Data Format register
 * @param[in]: Self Test
 * 				ADXL345_DATA_SELFTEST_ENABLE, ADXL345_DATA_SELFTEST_DISABLE
 * 			   SPI Config
 * 				ADXL345_DATA_SPI_3, ADXL345_DATA_SPI_4
 * 			   Int Interrupt
 * 			   	ADXL345_DATA_INT_INVERT_ENABLE, ADXL345_DATA_INT_INVERT_DISABLE
 * 			   Full res
 * 			   	ADXL345_DATA_FULLRES_ENABLE, ADXL345_DATA_FULLRES_DISABLE
 * 			   Justify
 * 			   	ADXL345_DATA_JUSTIFY_LEFT, ADXL345_DATA_JUSTIFY_RIGHT
 * 			   Range
 * 			   	ADXL345_DATA_RANGE_2G
 * 			   	ADXL345_DATA_RANGE_4G
 * 			   	ADXL345_DATA_RANGE_8G
 * 			   	ADXL345_DATA_RANGE_18G
 * @param[out]: none
 */
void adxl345_WriteDataFormat(uint8_t selftest, uint8_t spi, uint8_t intinv, uint8_t fullres, uint8_t justify, uint8_t range){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_DATA_FORMAT, selftest|spi|intinv|0x00|fullres|justify|range);
}

/*
 * @brief: Read XYZ data for adxl345
 * @param[in]: ptr to x,y,z variables
 * @param[out]: none
 */
void adxl345_ReadXYZ(int16_t *xdata, int16_t *ydata, int16_t *zdata){
	uint8_t b[6];
	IMU::i2c_ReadBuf(I2C_ID_ADXL345, ADXL345_RA_DATAX0, 6, b);
	*xdata = (int16_t)((uint16_t)b[0]<<8|(uint16_t)b[1]);
	*ydata = (int16_t)((uint16_t)b[2]<<8|(uint16_t)b[3]);
	*zdata = (int16_t)((uint16_t)b[4]<<8|(uint16_t)b[5]);
}

/*
 * @brief: Read FIFO ctl register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadFIFOCtl(uint8_t *fifo){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_FIFO_CTL,fifo);
}

/*
 * @brief: Write to FIFO ctl registers
 * @param[in]: FIFO
 * 				ADXL345_FIFO_BYPASS
 * 				ADXL345_FIFO_FIFO
 * 				ADXL345_FIFO_STREAM
 * 				ADXL345_FIFO_TRIGGER
 * 			  Trigger
 * 			  	ADXL345_TRIG_INT1, ADXL345_TRIG_INT2
 * 			  Sample
 * 			  	ADXL345_SAMPLE_WATERMARKENABLE
 * 			  	ADXL345_SAMPLE_WATERMARKDISABLE
 * 	@param[out]: none
 */
void adxl345_WriteFIFOCtl(uint8_t fifo, uint8_t trigger, uint8_t sample){
	IMU::i2c_WriteByte(I2C_ID_ADXL345, ADXL345_RA_FIFO_CTL, fifo|trigger|sample);
}

/*
 * @brief: Read FIFO status register
 * @param[in]: ptr to variable
 * @param[out]: none
 */
void adxl345_ReadFIFOStatus(uint8_t *fifost){
	IMU::i2c_ReadByte(I2C_ID_ADXL345, ADXL345_RA_FIFO_STATUS,fifost);
}
