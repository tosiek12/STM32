/*-----------------------------------------------------------------------*/
/* MMC/SDC (in SPI mode) control module  (C)ChaN, 2007                   */
/*-----------------------------------------------------------------------*/
/* Only rcvr_spi(), xmit_spi(), disk_timerproc() and some macros         */
/* are platform dependent.                                               */
/*-----------------------------------------------------------------------*/
#include "SD/spi_sd.h"
#include "cmsis/stm32f4xx.h"
#include "SD/diskio.h"
#include "Delay/delay.h"

/* Definitions for MMC/SDC command */
#define CMD0    (0x40+0)    /* GO_IDLE_STATE */
#define CMD1    (0x40+1)    /* SEND_OP_COND */
#define CMD8    (0x40+8)    /* SEND_IF_COND */
#define CMD9    (0x40+9)    /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */

/*--------------------------------------------------------------------------

 Module Private Functions

 ---------------------------------------------------------------------------*/

//enum { TRUE = 1, FALSE = 0 } bool;
static volatile DSTATUS Stat = STA_NOINIT; /* Disk status */

static volatile BYTE Timer1, Timer2; /* 100Hz decrement timer */

static BYTE CardType; /* b0:MMC, b1:SDC, b2:Block addressing */

static BYTE PowerFlag = 0; /* indicates if "power" is on */

#ifdef __cplusplus
extern "C" {
#endif

static void SELECT(void) {
	GPIOB->BSRRH |= GPIO_BSRR_BS_1; // CS w stan niski
}

static void DESELECT(void) {
	GPIOB->BSRRL |= GPIO_BSRR_BS_1;// CS w stan wysoki
}

void SPI_SD_Init(void) {
	//enable clk for SPI2 and GPIOB.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	// GPIOB - SCK, MISO, MOSI
	GPIOB->MODER |= GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15;
	GPIOB->AFR[1] = 0x55500000;

	// GPIOB - PB1( CS )
	GPIOB->MODER |= GPIO_MODER_MODER1_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;

	// init spi2
	//reset peripheral
	RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
	Delay::delay_ms(10);
	RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

	//config SPI2
	SPI2->CR1 |= SPI_CR1_MSTR	// master configuration
			| SPI_CR1_SSM 	// software slave managment enabled
			| SPI_CR1_SSI	//value of this bit is forced to NSS pin.
			| SPI_CR1_CPHA	//the second clk transistion is the first data capture edge
			| SPI_CR1_CPOL	//CK to 1 when idle
			;	//001 fpclk/2
	SPI2->CR1 |= SPI_CR1_SPE;	//peripheral enabled

	DESELECT();
}

static void xmit_spi(BYTE Data)  // Wyslanie bajtu do SD
		{
	while (!( SPI2->SR & SPI_SR_TXE))
		;
	SPI2->DR = Data;
}

static BYTE rcvr_spi(void) 		// Odebranie bajtu z SD
		{
	uint8_t Data = 0;

	while (!( SPI2->SR & SPI_SR_TXE))
		;
	SPI2->DR = 0xFF;
	while (!( SPI2->SR & SPI_SR_RXNE))
		;
	Data = SPI2->DR;

	return Data;
}

static void rcvr_spi_m(BYTE *dst) {
	*dst = rcvr_spi();
}

/*-----------------------------------------------------------------------*/
/* Wait for card ready                                                   */
/*-----------------------------------------------------------------------*/

static BYTE wait_ready(void) /* 1:Ready, 0:Timeout */
{
	BYTE res;

	Timer2 = 50; /* Wait for ready in timeout of 500ms */
	rcvr_spi();
	do
		res = rcvr_spi();
	while ((res != 0xFF) && Timer2);

	return res;
}

/*-----------------------------------------------------------------------*/
/* Power Control  (Platform dependent)                                   */
/*-----------------------------------------------------------------------*/
/* When the target system does not support socket power control, there   */
/* is nothing to do in these functions and chk_power always returns 1.   */

static void power_on(void) {
	uint8_t i;

	DESELECT();      // CS = 1

	//Wyslanie 10 razy 0xFF co daje ponad 80 (>74) cykle zegara
	//wymagane przez specyfikacje SD
	for (i = 0; i < 10; i++)
		xmit_spi(0xFF);

	PowerFlag = 1;
}

static void power_off(void) {
	PowerFlag = 0;
}

static int chk_power(void) /* Socket power state: 0=off, 1=on */
{
	return PowerFlag;
}

/*-----------------------------------------------------------------------*/
/* Receive a data packet from MMC                                        */
/*-----------------------------------------------------------------------*/

static bool rcvr_datablock(BYTE *buff, /* Data buffer to store received data */
UINT btr /* Byte count (must be even number) */
) {
	BYTE token;

	Timer1 = 10;
	do { /* Wait for data packet in timeout of 100ms */
		token = rcvr_spi();
	} while ((token == 0xFF) && Timer1);
	if (token != 0xFE)
		return 0; /* If not valid data token, retutn with error */

	do { /* Receive the data block into buffer */
		rcvr_spi_m(buff++);
		rcvr_spi_m(buff++);
	} while (btr -= 2);
	rcvr_spi(); /* Discard CRC */
	rcvr_spi();

	return 1; /* Return with success */
}

/*-----------------------------------------------------------------------*/
/* Send a data packet to MMC                                             */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
/* 1:OK, 0:Failed */
static bool xmit_datablock(const BYTE *buff, /* 512 byte data block to be transmitted */
BYTE token /* Data/Stop token */
) {
	BYTE resp = 0, wc;
	uint32_t i = 0;

	if (wait_ready() != 0xFF)
		return 0;

	xmit_spi(token); /* Xmit data token */
	if (token != 0xFD) { /* Is data token */
		wc = 0;
		do { /* Xmit the 512 byte data block to MMC */
			xmit_spi(*buff++);
			xmit_spi(*buff++);
		} while (--wc);

		rcvr_spi();
		rcvr_spi();

		while (i <= 64) {
			resp = rcvr_spi(); /* Reveive data response */
			if ((resp & 0x1F) == 0x05) /* If not accepted, return with error */
				break;
			i++;
		}
		while (rcvr_spi() == 0)
			;
	}
	if ((resp & 0x1F) == 0x05)
		return 1;
	else
		return 0;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd(BYTE cmd, /* Command byte */
DWORD arg /* Argument */
) {
	BYTE n, res;

	if (wait_ready() != 0xFF)
		return 0xFF;

//	if (cmd & 0x80) { /* Send a CMD55 prior to ACMD<n> */
//			cmd &= 0x7F;
//			res = send_cmd(CMD55, 0);
//			if (res > 1)
//				return res;
//		}
//
//		/* Select the card and wait for ready except to stop multiple block read */
//		if (cmd != CMD12) {
//			DESELECT();
//			SELECT();
//			if (!wait_ready())
//				return 0xFF;
//		}

	/* Send command packet */
	xmit_spi(cmd); /* Command */
	xmit_spi((BYTE) (arg >> 24)); /* Argument[31..24] */
	xmit_spi((BYTE) (arg >> 16)); /* Argument[23..16] */
	xmit_spi((BYTE) (arg >> 8)); /* Argument[15..8] */
	xmit_spi((BYTE) arg); /* Argument[7..0] */
	n = 0;
	if (cmd == CMD0)
		n = 0x95; /* CRC for CMD0(0) */
	if (cmd == CMD8)
		n = 0x87; /* CRC for CMD8(0x1AA) */
	xmit_spi(n);

	/* Receive command response */
	if (cmd == CMD12)
		rcvr_spi(); /* Skip a stuff byte when stop reading */
	n = 10; /* Wait for a valid response in timeout of 10 attempts */
	do
		res = rcvr_spi();
	while ((res & 0x80) && --n);

	return res; /* Return with the response value */
}

/*--------------------------------------------------------------------------

 Public Functions

 ---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(BYTE pdrv /* Physical drive nmuber (0) */
) {
	BYTE n, ty, ocr[4];

	if (pdrv)
		return STA_NOINIT; /* Supports only single drive */
	if (Stat & STA_NODISK)
		return Stat; /* No card in the socket */

	power_on(); /* Force socket power on */
	//send_initial_clock_train();

	SELECT(); /* CS = L */
	ty = 0;
	if (send_cmd(CMD0, 0) == 1) { /* Enter Idle state */
		Timer1 = 100; /* Initialization timeout of 1000 msec */
		if (send_cmd(CMD8, 0x1AA) == 1) { /* SDC Ver2+ */
			for (n = 0; n < 4; n++)
				ocr[n] = rcvr_spi();/* Get 32 bit return value of R7 resp */
			if (ocr[2] == 0x01 && ocr[3] == 0xAA) { /* The card can work at vdd range of 2.7-3.6V */
				do {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 1UL << 30) == 0)
						break; /* ACMD41 with HCS bit */
				} while (Timer1);
				if (Timer1 && send_cmd(CMD58, 0) == 0) { /* Check CCS bit */
					for (n = 0; n < 4; n++)
						ocr[n] = rcvr_spi();
					ty = (ocr[0] & 0x40) ? 6 : 2; /* Card id SDv2 */
				}
			}
		} else { /* SDC Ver1 or MMC */
			ty = (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */
			do {
				if (ty == 2) {
					if (send_cmd(CMD55, 0) <= 1 && send_cmd(CMD41, 0) == 0)
						break; /* ACMD41 */
				} else {
					if (send_cmd(CMD1, 0) == 0)
						break; /* CMD1 */
				}
			} while (Timer1);
			if (!Timer1 || send_cmd(CMD16, 512) != 0) /* Select R/W block length */
				ty = 0;
		}
	}
	CardType = ty;
	DESELECT(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	if (ty) /* Initialization succeded */
		Stat &= ~STA_NOINIT; /* Clear STA_NOINIT */
	else
		/* Initialization failed */
		power_off();

	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(BYTE pdrv /* Physical drive nmuber (0) */
) {
	if (pdrv)
		return STA_NOINIT; /* Supports only single drive */
	return Stat;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(BYTE pdrv, /* Physical drive nmuber (0) */
BYTE *buff, /* Pointer to the data buffer to store read data */
DWORD sector, /* Start sector number (LBA) */
UINT count /* Sector count (1..255) */
) {
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;

	if (!(CardType & 4))
		sector *= 512; /* Convert to byte address if needed */

	SELECT(); /* CS = L */

	if (count == 1) { /* Single block read */
		if ((send_cmd(CMD17, sector) == 0) /* READ_SINGLE_BLOCK */
		&& rcvr_datablock(buff, 512))
			count = 0;
	} else { /* Multiple block read */
		if (send_cmd(CMD18, sector) == 0) { /* READ_MULTIPLE_BLOCK */
			do {
				if (!rcvr_datablock(buff, 512))
					break;
				buff += 512;
			} while (--count);
			send_cmd(CMD12, 0); /* STOP_TRANSMISSION */
		}
	}

	DESELECT(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT disk_write(BYTE pdrv, /* Physical drive nmuber (0) */
const BYTE *buff, /* Pointer to the data to be written */
DWORD sector, /* Start sector number (LBA) */
UINT count /* Sector count (1..255) */
) {
	if (pdrv || !count)
		return RES_PARERR;
	if (Stat & STA_NOINIT)
		return RES_NOTRDY;
	if (Stat & STA_PROTECT)
		return RES_WRPRT;

	if (!(CardType & 4))
		sector *= 512; /* Convert to byte address if needed */

	SELECT(); /* CS = L */

	if (count == 1) { /* Single block write */
		if ((send_cmd(CMD24, sector) == 0) /* WRITE_BLOCK */
		&& xmit_datablock(buff, 0xFE))
			count = 0;
	} else { /* Multiple block write */
		if (CardType & 2) {
			send_cmd(CMD55, 0);
			send_cmd(CMD23, count); /* ACMD23 */
		}
		if (send_cmd(CMD25, sector) == 0) { /* WRITE_MULTIPLE_BLOCK */
			do {
				if (!xmit_datablock(buff, 0xFC))
					break;
				buff += 512;
			} while (--count);
			if (!xmit_datablock(0, 0xFD)) /* STOP_TRAN token */
				count = 1;
		}
	}

	DESELECT(); /* CS = H */
	rcvr_spi(); /* Idle (Release DO) */

	return count ? RES_ERROR : RES_OK;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(BYTE pdrv, /* Physical drive nmuber (0) */
BYTE ctrl, /* Control code */
void *buff /* Buffer to send/receive control data */
) {
	DRESULT res;
	BYTE n, csd[16], *ptr = (BYTE*) buff;
	WORD csize;

	if (pdrv)
		return RES_PARERR;

	res = RES_ERROR;

	if (ctrl == CTRL_POWER) {
		switch (*ptr) {
		case 0: /* Sub control code == 0 (POWER_OFF) */
			if (chk_power())
				power_off(); /* Power off */
			res = RES_OK;
			break;
		case 1: /* Sub control code == 1 (POWER_ON) */
			power_on(); /* Power on */
			res = RES_OK;
			break;
		case 2: /* Sub control code == 2 (POWER_GET) */
			*(ptr + 1) = (BYTE) chk_power();
			res = RES_OK;
			break;
		default:
			res = RES_PARERR;
		}
	} else {
		if (Stat & STA_NOINIT)
			return RES_NOTRDY;

		SELECT(); /* CS = L */

		switch (ctrl) {
		case GET_SECTOR_COUNT: /* Get number of sectors on the disk (DWORD) */
			if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
				if ((csd[0] >> 6) == 1) { /* SDC ver 2.00 */
					csize = csd[9] + ((WORD) csd[8] << 8) + 1;
					*(DWORD*) buff = (DWORD) csize << 10;
				} else { /* MMC or SDC ver 1.XX */
					n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
					csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
					*(DWORD*) buff = (DWORD) csize << (n - 9);
				}
				res = RES_OK;
			}
			break;

		case GET_SECTOR_SIZE: /* Get sectors on the disk (WORD) */
			*(WORD*) buff = 512;
			res = RES_OK;
			break;

		case CTRL_SYNC: /* Make sure that data has been written */
			if (wait_ready() == 0xFF)
				res = RES_OK;
			break;

		case MMC_GET_CSD: /* Receive CSD as a data block (16 bytes) */
			if (send_cmd(CMD9, 0) == 0 /* READ_CSD */
			&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_CID: /* Receive CID as a data block (16 bytes) */
			if (send_cmd(CMD10, 0) == 0 /* READ_CID */
			&& rcvr_datablock(ptr, 16))
				res = RES_OK;
			break;

		case MMC_GET_OCR: /* Receive OCR as an R3 resp (4 bytes) */
			if (send_cmd(CMD58, 0) == 0) { /* READ_OCR */
				for (n = 0; n < 4; n++)
					*ptr++ = rcvr_spi();
				res = RES_OK;
			}
			break;
//        case MMC_GET_TYPE :    /* Get card type flags (1 byte) */
//            *ptr = CardType;
//            res = RES_OK;
//            break;

		default:
			res = RES_PARERR;
		}

		DESELECT(); /* CS = H */
		rcvr_spi(); /* Idle (Release DO) */
	}

	return res;
}

/*-----------------------------------------------------------------------*/
/* Device Timer Interrupt Procedure  (Platform dependent)                */
/*-----------------------------------------------------------------------*/
/* This function must be called in period of 10ms                        */

void disk_timerproc(void) {
//    BYTE n, s;
	BYTE n;

	n = Timer1; /* 100Hz decrement timer */
	if (n)
		Timer1 = --n;
	n = Timer2;
	if (n)
		Timer2 = --n;

}

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD get_fattime(void) {

	return ((2011UL - 1980) << 25)    // Year = 2011
	| (1UL << 21)            // Month = January
			| (1UL << 16)            // Day = 1
			| (12U << 11)            // Hour = 12
			| (0U << 5)              // Min = 00
			| (0U >> 1)              // Sec = 00
	;

}

void testSDInit() {
	uint8_t n;
	// start MMC in SPI mode
	DESELECT(); /* CS = H */
	for (n = 10; n; n--) {
		xmit_spi(0xFF);	 // send 10*8=80 clock pulses
	}

	//send the CMD0 to reset the card:
	volatile uint8_t i;

	SELECT(); /* CS = L */
	xmit_spi(0x40);    // cmd0
	xmit_spi(0x00);
	xmit_spi(0x00);
	xmit_spi(0x00);
	xmit_spi(0x00);
	xmit_spi(0x95);    // precalculated checksum as we are still in MMC mode

	xmit_spi(0xFF);	//multiple of dummy clk

	// wait for the repsonse (response[7] == 0)
	for (n = 0; n < 200; n++) {	//czeka na 0x01
		i = rcvr_spi();
		if (!(i & 0x80)) {
			DESELECT(); /* CS = H */
			xmit_spi(0xFF);

			break;
		}
	}

	if (i == 1) {
		// MMC found, continue
		i = 100;
	} else {
		// no response from MMC, abort
		i = 0;
	}

}

#ifdef __cplusplus
}
#endif
