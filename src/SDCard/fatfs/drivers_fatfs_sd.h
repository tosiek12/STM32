/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef _DRIVERS_DISKIO_DEFINED_SD
#define _DRIVERS_DISKIO_DEFINED_SD

#define _USE_WRITE	1	/* 1: Enable disk_write function */
#define _USE_IOCTL	1	/* 1: Enable disk_ioctl fucntion */

#include "diskio.h"
#include "integer.h"

#include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
//#include "misc.h"
//#include "../src/Delay/Delay.h"

extern volatile uint16_t TimeSPI;

#ifndef FATFS_CS_PIN
#define FATFS_CS_RCC						RCC_AHB1Periph_GPIOB			
#define FATFS_CS_PORT						GPIOB
#define FATFS_CS_PIN						GPIO_PIN_1
#endif

#ifndef FATFS_USE_DETECT_PIN
#define FATFS_USE_DETECT_PIN				0
#endif

#ifndef FATFS_USE_WRITEPROTECT_PIN
#define FATFS_USE_WRITEPROTECT_PIN			0
#endif

#if FATFS_USE_DETECT_PIN > 0
#ifndef FATFS_USE_DETECT_PIN_PIN
#define FATFS_USE_DETECT_PIN_RCC			RCC_AHB1Periph_GPIOB			
#define FATFS_USE_DETECT_PIN_PORT			GPIOB
#define FATFS_USE_DETECT_PIN_PIN			GPIO_PIN_6
#endif
#endif

#if FATFS_USE_WRITEPROTECT_PIN > 0
#ifndef FATFS_USE_WRITEPROTECT_PIN_PIN
#define FATFS_USE_WRITEPROTECT_PIN_RCC		RCC_AHB1Periph_GPIOB			
#define FATFS_USE_WRITEPROTECT_PIN_PORT		GPIOB
#define FATFS_USE_WRITEPROTECT_PIN_PIN		GPIO_PIN_7
#endif
#endif

#define FATFS_CS_LOW						FATFS_CS_PORT->BSRRH = FATFS_CS_PIN
#define FATFS_CS_HIGH						FATFS_CS_PORT->BSRRL = FATFS_CS_PIN

/*---------------------------------------*/
/* Prototypes for disk control functions */
DSTATUS TM_FATFS_SD_disk_initialize(void);
DSTATUS TM_FATFS_SD_disk_status(void);
DRESULT TM_FATFS_SD_disk_read(BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_write(const BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_ioctl(BYTE cmd, void* buff);

#endif

