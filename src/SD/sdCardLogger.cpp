#include "SD/sdCardLogger.h"
#include "SD/ff.h"
#include "SD/spi_sd.h"
#include "stm32f4xx_hal.h"
#include "main.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}
#include "SpeedTester/speedTester.h"

SdCardLogger sdCardLogger;

FRESULT SdCardLogger::TM_FATFS_DriveSize(uint32_t* total, uint32_t* free) {
	FATFS *fs;
	DWORD fre_clust;
	FRESULT res;

	/* Get volume information and free clusters of drive */
	res = f_getfree("0:", &fre_clust, &fs);
	if (res != FR_OK) {
		return res;
	}

	/* Get total sectors and free sectors */
	*total = (fs->n_fatent - 2) * fs->csize / 2;
	*free = fre_clust * fs->csize / 2;

	/* Return OK */
	return FR_OK;
}

FRESULT SdCardLogger::TM_FATFS_USBDriveSize(uint32_t* total, uint32_t* free) {
	FATFS *fs;
	DWORD fre_clust;
	FRESULT res;

	/* Get volume information and free clusters of drive */
	res = f_getfree("1:", &fre_clust, &fs);
	if (res != FR_OK) {
		return res;
	}

	/* Get total sectors and free sectors */
	*total = (fs->n_fatent - 2) * fs->csize / 2;
	*free = fre_clust * fs->csize / 2;

	/* Return OK */
	return FR_OK;
}

void SdCardLogger::testCreatingFiles(const uint8_t *pfilename) {
	//Mount drive
	state = f_mount(&FatFs, "", 0);
	if (state == FR_OK) {
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, "(1):mount ok");
		state = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);		//Try to open file
		sprintf((char *)buff, "(%d): file open. State(%d)", 1, state);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led
			numberOfChars = f_puts("First string in my file\n", &fil);			//If we put more than 0 characters (everything OK)
			if (numberOfChars > 0) {
				sprintf((char *)buff, "(1):written %d chars.", numberOfChars);
				VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
				state = TM_FATFS_DriveSize(&total, &free);
				if (state == FR_OK) {
					//Data for drive size are valid
				}
			}
			state = f_sync(&fil);
			sprintf((char *)buff, "(%d): flushed. State(%d)", 1, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);

			state = f_close(&fil);			//Close file, don't forget this!
			sprintf((char *)buff, "(%d): closed. State(%d)", 1, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		}

		state = f_open(&fil, "test.txt", FA_OPEN_EXISTING | FA_WRITE);		//Try to open file
		sprintf((char *)buff, "(%d): file open. State(%d)", 2, state);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		if (state == FR_OK) {
			state = f_rename("test.txt", (const char *)pfilename);
			sprintf((char *)buff, "(%d): Rename. State(%d)", 2, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
			if (state == FR_OK) {
				numberOfChars = f_puts("First string in my file1212\n", &fil);			//If we put more than 0 characters (everything OK)
				if (numberOfChars > 0) {
					sprintf((char *)buff, "(2):written %d chars.", numberOfChars);
					VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
					if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
						//Data for drive size are valid
					}
				}
			}
			state = f_sync(&fil);
			sprintf((char *)buff, "(%d): flushed. State(%d)", 2, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);

			state = f_close(&fil);			//Close file, don't forget this!
			sprintf((char *)buff, "(%d): closed. State(%d)", 2, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		}
		f_mount(0, "", 1);		//Unmount drive, don't forget this!
	}
}

void SdCardLogger::testCreatingFiles2(void) {
	FIL plik;
	UINT zapisanych_bajtow = 0;
	uint16_t i;

	strncpy((char *)buff,"123456789abcdef\r\n",strlen("123456789abcdef\r\n"));

	state = f_mount(&FatFs, "", 0);
		if( state == FR_OK ) {
			state = f_open(&plik, "plik.txt", FA_CREATE_ALWAYS | FA_WRITE );
			if( state == FR_OK ) {
				for( i = 0; i < 1000; i++ ) {
					state = f_write( &plik, buff, 17, &zapisanych_bajtow );
					if(state != FR_OK) {
						for(;;) {
							//inf error loop ;(
						}
					}
				}
			}
			state = f_close( &plik );
		}
	f_mount(0, "", 1);
}

void SdCardLogger::initialize(void) {
	SPI_SD_Init();
	flagsComunicationInterface[f_interface_sdCard] = f_configured;
}

void SdCardLogger::test(void) {
	//inne
	uint32_t cnt[5] = { 0 };
	uint32_t numberOfChars;
	int16_t i = 0;
	int16_t numberOfSendChars = -1;

	VCP_writeStringFrame(frameAddress_Pecet, frameType_Log,"Start: Testing SD-Card\n");
	//testCreatingFiles(s_RxFrameBuffer.Msg);

	speedTester.tic();
	state = f_mount(&FatFs, "", 0);
	cnt[0] = speedTester.toc();

	sprintf((char *)buff, "(%d): disc mount. State(%d)", 1, state);
	VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
	if (state == FR_OK) {
		speedTester.tic();
		state = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);		//Try to open file
		cnt[1] = speedTester.toc();
		sprintf((char *)buff, "(%d): file open. State(%d)", 1, state);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led
			numberOfChars = 0;
			speedTester.tic();
			for( i = 0; i < 1000; i++ ) {
				numberOfSendChars = f_puts("First string in my file\n", &fil);			//If we put more than 0 characters (everything OK)
				if(numberOfSendChars == -1) {
					++numberOfChars;
				}
			}
			cnt[2] = speedTester.toc();
			sprintf((char *)buff, "(1):written failed %lu/1000 times.", numberOfChars);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);

			speedTester.tic();
			state = f_sync(&fil);
			cnt[3] = speedTester.toc();

			sprintf((char *)buff, "(%d): flushed. State(%d)", 1, state);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		}
		speedTester.tic();
		state = f_close(&fil);			//Close file, don't forget this!
		cnt[4] = speedTester.toc();

		sprintf((char *)buff, "(%d): closed. State(%d)", 1, state);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
	}
	f_mount(0, "", 1);		//Unmount drive, don't forget this!

	numberOfChars = sprintf((char *) buff, "Time: %lu,%lu,%lu,%lu,%lu\n", cnt[0], cnt[1], cnt[2], cnt[3], cnt[4]);
	VCP_writeStringFrame(frameAddress_Pecet, frameType_FunctionTest, buff);
	VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, "End: Testing SD-Card\n");
}

void SdCardLogger::openFileForIMU(const char *pFilename){
//Mount drive
	uint8_t buf[50];
	if(IMU_state == f_deviceWorking) {
		return;
	}
	state = f_mount(&FatFs_SDCard, "", 0);
	if (state == FR_OK) {
		flagsComunicationInterface[f_interface_sdCard] = f_connected;
		IMU_state = f_connected;
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, "(1):mount ok");
		/* Opens an existing file. If not exist, creates a new file. */
		state = f_open(&IMU_dataFile, pFilename, FA_CREATE_ALWAYS | FA_WRITE);
		IMU_state = f_error;
		sprintf((char *)buff, "(%d): file open. State(%d)", 1, state);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		if (state == FR_OK) {
			/* Seek to end of the file to append data */
			state = f_lseek(&IMU_dataFile, f_size(&IMU_dataFile));
			if (state != FR_OK) {
				f_close(&IMU_dataFile);
				IMU_state = f_error;
			} else {
				IMU_state = f_deviceWorking;
				//append some header data,
				f_printf(&IMU_dataFile, "\nNew session opened\n");
				sprintf((char*) buf, "%s,%s,%s\n","X_Roll","Y_Pitch","Z_Yaw");
				sdCardLogger.writeStringForIMU((char *) buf);
			}
		}
	}
}
void SdCardLogger::closeFileForIMU(void){
	state = f_close(&IMU_dataFile);
	sprintf((char *)buff, "Closed. State(%d)", state);
	VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);

	state = f_mount(0, "", 1);
	sprintf((char *)buff, "Unmounded. State(%d)", state);
	VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
	IMU_state = f_nonInit;
	flagsComunicationInterface[f_interface_sdCard] = f_configured;

}

int16_t SdCardLogger::writeStringForIMU(const char *pString){
	if(isUsed == 1 || IMU_state != f_deviceWorking) {
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, "File in use, or not opened");
		return  -1;
	} else {
		numberOfChars = f_puts(pString, &IMU_dataFile);			//If we put more than 0 characters (everything OK)
		if (numberOfChars > 0) {
			sprintf((char *)buff, "Written %d chars.", numberOfChars);
			VCP_writeStringFrame(frameAddress_Pecet, frameType_Log, buff);
		}
		return numberOfChars;
	}
}
