#ifndef SDCARDLOGGER_H_
#define SDCARDLOGGER_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "stm32f4xx.h"
#include "SD/ff.h"
#include "main.h"

class SdCardLogger {
private:

	/* IMU data file for logging */
	FATFS FatFs_SDCard;
	FIL IMU_dataFile;	//File object
	uint8_t IMU_state;

	/* Statistics */
	uint8_t isUsed;

	/* Temp */
	FATFS FatFs;
	FIL fil;	//File object
	FRESULT state;
	uint32_t total, free;	//Free and total space
	int8_t numberOfChars;
	uint8_t buff[50];
public:
	SdCardLogger() {
		isUsed = 0;
		numberOfChars = -1;
		state = FR_INT_ERR;
		IMU_state = f_nonInit;
	}
	/**
	 * Get SD card drive size
	 *
	 * Parameters:
	 * 	- uint32_t* total: pointer to variable to store total size of card
	 * 	- uint32_t* free: pointer to variable to store free space on card
	 *
	 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
	 */
	FRESULT TM_FATFS_DriveSize(uint32_t* total, uint32_t* free);

	/**
	 * Get SD card drive size
	 *
	 * Parameters:
	 * 	- uint32_t* total: pointer to variable to store total size of card
	 * 	- uint32_t* free: pointer to variable to store free space on card
	 *
	 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
	 */
	FRESULT TM_FATFS_USBDriveSize(uint32_t* total, uint32_t* free);

	void initialize(void);
	void testCreatingFiles(const uint8_t *pfilename);
	void testCreatingFiles2(void);
	void test(void);
	void openFileForIMU(const char *pFilename);
	void closeFileForIMU(void);
	int16_t writeStringForIMU(const char *pString);
};

extern SdCardLogger sdCardLogger;
#endif

