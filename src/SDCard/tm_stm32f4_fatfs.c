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
#include "SDCard/tm_stm32f4_fatfs.h"

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

void testSD() {
	FATFS FatFs;
	//File object
	FIL fil;
	//Free and total space
	uint32_t total, free;
	volatile uint8_t state = 0;
	//Mount drive
	state = f_mount(&FatFs, "", 0);
	if (state == FR_OK) {
		//Mounted OK, turn on RED LED

		//Try to open file
		state = f_open(&fil, "1stfile", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led

			//If we put more than 0 characters (everything OK)
			state = f_puts("First string in my file\n", &fil);
			if (state > 0) {
				state = TM_FATFS_DriveSize(&total, &free);
				if (state == FR_OK) {
					//Data for drive size are valid
				}

				//Turn on both leds
			}

			//Close file, don't forget this!
			f_close(&fil);
		}

		//Try to open file
		if (f_open(&fil, "2ndfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
			//File opened, turn off RED and turn on GREEN led

			//If we put more than 0 characters (everything OK)
			if (f_puts("First string in my file\n", &fil) > 0) {
				if (TM_FATFS_DriveSize(&total, &free) == FR_OK) {
					//Data for drive size are valid
				}

				//Turn on both leds
			}

			//Close file, don't forget this!
			f_close(&fil);
		}

		//Unmount drive, don't forget this!
		f_mount(0, "", 1);
	}
}

