/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include "SD/ff.h"
#include "SD/tm_stm32f4_fatfs.h"


static FATFS FatFs;
//File object
static FIL fil;

FRESULT TM_FATFS_DriveSize(uint32_t* total, uint32_t* free) {
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

FRESULT TM_FATFS_USBDriveSize(uint32_t* total, uint32_t* free) {
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

void testCreatingFiles() {
	uint32_t total, free;	//Free and total space
	FRESULT state = FR_INT_ERR;
	int8_t numberOfChars = -1;

	//Mount drive
	state = f_mount(&FatFs, "", 0);
	if (state == FR_OK) {
		state = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);		//Try to open file
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led
			numberOfChars = f_puts("First string in my file\n", &fil);			//If we put more than 0 characters (everything OK)
			if (numberOfChars > 0) {
				state = TM_FATFS_DriveSize(&total, &free);
				if (state == FR_OK) {
					//Data for drive size are valid
				}
			}
			state = f_close(&fil);			//Close file, don't forget this!
		}

		state = f_open(&fil, "test.txt", FA_OPEN_EXISTING | FA_WRITE);		//Try to open file
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led
			state = f_rename("test.txt", "test123.txt");
			numberOfChars = f_puts("First string in my file\n", &fil);			//If we put more than 0 characters (everything OK)
			if (numberOfChars > 0) {
				if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
					//Data for drive size are valid
				}
			}
			state = f_close(&fil);			//Close file, don't forget this!
		}
		f_mount(0, "", 1);		//Unmount drive, don't forget this!
	}
}

void testCreatingFiles2() {
	FATFS fatfs;
	FIL plik;
	FRESULT fresult;
	UINT zapisanych_bajtow = 0;
	char buffor[30] = "123456789abcdef\r\n";
	uint16_t i;

	fresult = f_mount(&fatfs, "", 0);
		if( fresult == FR_OK ) {
			fresult = f_open(&plik, "plik.txt", FA_CREATE_ALWAYS | FA_WRITE );
			if( fresult == FR_OK ) {
				for( i = 0; i < 1000; i++ ) {
					fresult = f_write( &plik, buffor, 17, &zapisanych_bajtow );
					if(fresult != FR_OK) {
						for(;;) {
							//inf error loop ;(
						}
					}
				}
			}
			fresult = f_close( &plik );
		}
	f_mount(0, "", 1);
}
