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
	FIL IMU_dataFile;	//For measurement logging.
	FIL debug_dataFile;	//For debug logging.
	uint8_t IMU_state;

	/* Statistics */
	uint8_t isUsed;

	/* Temp */
	FATFS FatFs;
	FIL fil;	//File object
	FRESULT state;
	uint32_t total, // total size of card
	free;	//free space on card
	int8_t numberOfChars;
	uint8_t buff[50];
	char headerOfIMUFile[50];
public:
	SdCardLogger() {
		isUsed = 0;
		strncpy(headerOfIMUFile,"x,y,z,x_c,y_c,z_c,altG,lonG,latG,dop,hdop,altP\n",50);
		numberOfChars = -1;
		state = FR_INT_ERR;
		IMU_state = f_nonInit;
		flagsComunicationInterface[f_interface_sdCard] = f_nonInit;
		total = 0;
		free = 0;

	}

	enum e_File {
		measurement, debug
	};

	/**
	 * Get SD card drive size
	 *
	 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
	 */
	FRESULT TM_FATFS_DriveSize(void);

	/**
	 * Get SD card drive size
	 *
	 * Returns FRESULT struct members. If data are valid, FR_OK is returned.
	 */
	FRESULT TM_FATFS_USBDriveSize(void);

	void initialize(void);
	void testCreatingFiles(const uint8_t *pfilename);
	void testCreatingFiles2(void);
	void test(void);
	void openFile(e_File theFile, const char *pFilename);
	void closeFile(e_File theFile);
	int16_t writeStringForIMU(const char *pString);
	int16_t writeStringForDebug(const char *pString);

};

extern SdCardLogger sdCardLogger;
#endif

