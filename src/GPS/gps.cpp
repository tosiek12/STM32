#include "GPS/gps.h"
#include "10DOF/IMU.h"
#include "cortexm/ExceptionHandlers.h"
#include "main.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}

/* UART handler declaration */
UART_HandleTypeDef UartHandle;

uint8_t buf1[80] = { 0 };
uint8_t buf2[80] = { 0 };
uint8_t *pFrame = buf1; //Zmienia sie wskaznik i wartosci
uint16_t frameSize = 0;
uint8_t *pTempFrame = buf2;
uint8_t charsInFrame = 0;
uint8_t charToSkip = 0;

volatile uint8_t isNewFrame = 0;
struct gpsData_t gpsdata;

/* private functions: */
void GPS_PushCharToFrame(volatile uint8_t recievedChar);
void GPS_UnitTests();

/*
 * Decode frame received from GPS, and update data in IMU class.
 */

void GPS_PushCharToFrame(volatile uint8_t recievedChar) {
	if (charToSkip > 0) {
		--charToSkip;	//skip check Sum
		return;
	}

	if (recievedChar == '$') {
		memset(pTempFrame, 0, 80);	//new frame starts
		pTempFrame[0] = recievedChar;
		charsInFrame = 1;
	} else if (pTempFrame[0] == '$') {
		if (recievedChar == '*') {
			uint8_t *temp;	//frame finished
			//Skip Check Sum
			charToSkip = 2;

			//Swap buffers
			temp = pFrame;
			pFrame = pTempFrame;
			pTempFrame = temp;
			frameSize = charsInFrame;

			if (isNewFrame == 1) {
				//printf("overwrite\n");
			}
			isNewFrame = 1;

			charsInFrame = 0;
			pTempFrame[0] = '0';

			GPS_Parse(&gpsdata, pFrame, frameSize); //interpretuj ramke
			imu10DOF.updateGPSData(&gpsdata);
			isNewFrame = 0;
		} else if (recievedChar == '\n' || recievedChar == '\r') {
			charsInFrame = 0;
			pTempFrame[0] = '0';
			//printf("Enter\n");
		} else if (charsInFrame == 77) {
			//printf("ERROR - za duza ramka\n");
			charsInFrame = 0;
			pTempFrame[0] = '0';
		} else {
			pTempFrame[charsInFrame] = recievedChar;
			++charsInFrame;
		}
	} else {
		charsInFrame = 0;
		pTempFrame[0] = '0';
		//printf("Zly znak startu\n");
	}
}


static uint32_t GPS_atoi(const char *p)
{
	uint32_t out = 0;
	while ((*p >= '0' && *p <= '9') || *p == '.')
	{
		if (*p == '.') {
			p++;
			continue;
		}
		out *= 10;
		out += *p - '0';
		p++;
	}
	return out;
}

//Potrzebne parsowanie ramek:
//GGA - Global Positioning System Fix Data
//GSA - GNSS DOP and Active Satellites
//RMC - Recommended Minimum Specific GNSS Data
//VTG - Course Over Ground and Ground Speed
void GPS_Parse(struct gpsData_t *_gpsdata, uint8_t *buf, uint8_t len)
{
	char *p;
	uint32_t tmp, tmp2;
	p = (char *)buf;

	if (!strncmp(p, "$GPGGA", 6))
	{
		//123519 – Aktualność danych - 12:35 : 19 UTC,
		p += 7;
		_gpsdata->hour = (p[0] - '0') * 10 + p[1] - '0';
		_gpsdata->min = (p[2] - '0') * 10 + p[3] - '0';
		_gpsdata->sec = (p[4] - '0') * 10 + p[5] - '0';
		memcpy(_gpsdata->hhmmss, p, 6*sizeof(uint8_t));

		//4807.038, N – szerokość geograficzna(latitude) - 48 deg 07.038' N,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		tmp2 = tmp%10000;	//in current used GPS there is one more point at the end. not 3 as stated above.
		_gpsdata->lon = tmp - tmp2*0.4f;//change decimal values to degree (mod 60) - convention on google maps.

		//01131.000, E – długość geograficzna(longitude) - 11 deg 31.000' E,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		tmp2 = tmp%10000;	//in current used GPS there is one more point at the end. not 3 as stated above.
		_gpsdata->lon = tmp - tmp2*0.4f;//change decimal values to degree (mod 60) - convention on google maps.

		//1 – jakość pomiaru
		p = strchr(p, ',') + 1;
		_gpsdata->valid = (p[0] - '0') ? 1 : 0;

		//08 – ilość śledzonych satelitów,
		p = strchr(p, ',') + 1;
		_gpsdata->sats = (p[0] - '0') * 10 + p[1] - '0';

		//0.9 – horyzontalna dokładność pozycji(HDOP),
		p = strchr(p, ',') + 1;
		_gpsdata->hdop = GPS_atoi(p);

		//545.4, M – wysokość w metrach nad poziom morza,
		p = strchr(p, ',') + 1;
		_gpsdata->alt = GPS_atoi(p);
	}
	else if (!strncmp(p, "$GPRMC", 6)) {
//		3    = Latitude of fix
//		4    = N or S of longitude
//		5    = Longitude of fix
//		6    = E or W of longitude
//		7    = Speed over ground in knots
//		8    = Track made good in degrees True
//		9    = UTC date of fix
//		10   = Magnetic variation degrees (Easterly var. subtracts from true course)
//		11   = E or W of magnetic variation
//		12   = Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid)
//		13   = Checksum

		//		1    = UTC time of fix
		//123519 – Aktualność danych - 12:35:19 UTC,
		p += 7;
		_gpsdata->hour = (p[0] - '0') * 10 + p[1] - '0';
		_gpsdata->min = (p[2] - '0') * 10 + p[3] - '0';
		_gpsdata->sec = (p[4] - '0') * 10 + p[5] - '0';
		memcpy(_gpsdata->hhmmss, p, 6*sizeof(uint8_t));

		//		2    = Data status (A=Valid position, V=navigation receiver warning)
		// A – status(A – aktywny; V – nieaktywny),
		p = strchr(p, ',') + 1;
		_gpsdata->valid = (p[0] == 'A') ? 1 : 0;

		//4807.038, N – szerokość geograficzna(latitude) - 48 deg 07.038' N,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		tmp2 = tmp%10000;	//in current used GPS there is one more point at the end. not 3 as stated above.
		_gpsdata->lon = tmp - tmp2*0.4f;//change decimal values to degree (mod 60) - convention on google maps.

		//01131.000, E – długość geograficzna(longitude) - 11 deg 31.000' E,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		tmp2 = tmp%10000;	//in current used GPS there is one more point at the end. not 3 as stated above.
		_gpsdata->lon = tmp - tmp2*0.4f;//change decimal values to degree (mod 60) - convention on google maps.

		//022.4 – prędkość obiektu(liczona w węzłach),
		p = strchr(p, ',') + 1;
		//gpsdata->speedInKnots = GPS_atoi(p);

		//084.4 – kąt śledzenia / poruszania się obiektu(w stopniach)
		p = strchr(p, ',') + 1;
		_gpsdata->headingTruePRMC = GPS_atoi(p);

		//230394 – data(23 marca 1994),
		p = strchr(p, ',') + 1;
		_gpsdata->day = (p[0] - '0') * 10 + p[1] - '0';
		_gpsdata->month = (p[2] - '0') * 10 + p[3] - '0';
		_gpsdata->year = (p[4] - '0') * 10 + p[5] - '0';
	}
	else if (!strncmp(p, "$GPGSA", 6)) {
		//		1    = Mode:
		//		       M=Manual, forced to operate in 2D or 3D
		//		       A=Automatic, 3D/2D
		p += 7;
		_gpsdata->modeGPSA = p[0];

		//		2    = Mode:
		//		       1=Fix not available
		//		       2=2D
		//		       3=3D
		p = strchr(p, ',') + 1;
		_gpsdata->position3D = (p[0] - '0');

		//		3-14 = PRN's of Satellite Vechicles (SV's) used in position fix (null for unused fields)
		//04,05... – Numery satelitów wykorzystane do wyznaczenia pozycji (miejsce dla 12 satelitów),
		for (uint8_t it = 0; it < 12; it++) {
			p = strchr(p, ',') + 1;
			if(*p == ',') {
				_gpsdata->satID[it] = '-';
			} else {
				_gpsdata->satID[it] = (p[0] - '0') * 10 + p[1] - '0';
			}
		}

		//		15   = Position Dilution of Precision (PDOP)
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		_gpsdata->dop = tmp;

		//		16   = Horizontal Dilution of Precision (HDOP)
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		_gpsdata->hdop = tmp;

		//		17   = Vertical Dilution of Precision (VDOP)
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		_gpsdata->vdop = tmp;
	}
	else if (!strncmp(p, "$GPVTG", 6)) {
		//054.7,T – ścieżka poruszania się (w stopniach),
		p += 7;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'T')
			_gpsdata->headingTruePVTG= tmp;

		//034.4,M – ścieżka poruszania się (na podstawię współrzędnych magnetycznych – w stopniach),
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'M')
			_gpsdata->headingMagneticPVTG= tmp;

		//005.5,N – prędkość w węzłach,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'N')
			//gpsdata->speed = tmp;

		//010.2,K – prędkość w km/h,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'K')
			_gpsdata->speedInKm_h = tmp;
	}
}

static void assertTrue(bool testedValue) {
	if(!testedValue) {
		while(true) {
		}
	}

}

void GPS_UnitTests() {
	struct gpsData_t temp;

	GPS_Parse(&temp,(uint8_t *) "$GPRMC,225233.990,V,3939.4000,N,10506.4000,W,0.00,51.40,280804,,*35",80);
	//Test with asserts:
	assertTrue(temp.hour == 22);
	assertTrue(temp.min == 52);
	assertTrue(temp.sec == 33);

	assertTrue(temp.valid == 0);
	assertTrue(temp.lat == 39394000);
	assertTrue(temp.lon == -105064000);

	//gpsdata->speedInKnots = GPS_atoi(p);
	assertTrue(temp.headingTruePRMC == 5140);

	assertTrue(temp.day == 28);
	assertTrue(temp.month == 8);
	assertTrue(temp.year == 4);

	/**************************************************************************/
	GPS_Parse(&temp,(uint8_t *) "$GPGSA,A,3,11,29,07,08,19,28,26,,,,,,2.3,1.2,2.0*30",80);
	//Test with asserts:
	assertTrue(temp.modeGPSA== 'A');
	assertTrue(temp.position3D  == 3);

	assertTrue(temp.satID[0]== 11);
	assertTrue(temp.satID[1]== 29);
	assertTrue(temp.satID[11]== '-');

	assertTrue(temp.dop == 23);
	assertTrue(temp.hdop == 12);
	assertTrue(temp.vdop == 20);

	/**************************************************************************/
	// Parse the current position again
	GPS_Parse(&temp,(uint8_t *) "$GPGGA,170834,4124.8963,N,08151.6838,W,1,05,1.5,280.2,M,-34.0,M,,,*59",80);
	//Test with asserts:
	assertTrue(temp.hour == 17);
	assertTrue(temp.min == 8);
	assertTrue(temp.sec == 34);

	assertTrue(temp.lat == 41248963);
	assertTrue(temp.lon == -81516838);

	assertTrue(temp.valid == 1);
	assertTrue(temp.sats == 5);
	assertTrue(temp.hdop == 15);
	assertTrue(temp.alt == 2802);
	/**************************************************************************/
	// Parse the current position again
	GPS_Parse(&temp,(uint8_t *) "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48",80);
	//Test with asserts:
	assertTrue(temp.headingTruePVTG == 547);
	assertTrue(temp.headingMagneticPVTG == 344);

}

void GPS_Init() {
	/* Configure hardware */
	HAL_UART_MspInit(&UartHandle);

	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
	 - Word Length = 8 Bits
	 - Stop Bit = One Stop bit
	 - Parity = None
	 - BaudRate = 9600 baud
	 - Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;

	if (HAL_UART_Init(&UartHandle) != HAL_OK) {
		Error_Handler();
	}

	/* Enable the UART Parity Error Interrupt */
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_PE);
	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_ERR);
	/* Enable the UART Data Register not empty Interrupt */
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);

	//__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TXE);

	//HAL_UART_Transmit(&UartHandle,(unsigned char *)"", 5, 4000);

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(USARTx_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART2 clock */
	USARTx_CLK_ENABLE()
	;

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	/*##-1- Reset peripherals ##################################################*/
	USARTx_FORCE_RESET();
	USARTx_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
	/* Configure UART Rx as alternate function  */
	HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
	/* Peripheral interrupt Deinit*/
	HAL_NVIC_DisableIRQ(USARTx_IRQn);
}

extern "C" void USART1_IRQHandler(void) {
	volatile uint8_t data;
	HAL_NVIC_ClearPendingIRQ((IRQn_Type) USART1_IRQn);

	if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_ORE) != RESET) {
		__HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_ORE);
		data = (uint8_t) (USART1->SR & (uint8_t) 0xFF);  //read status
		//then read data register ...
		data = (uint8_t) (USART1->DR & (uint8_t) 0xFF);
		GPS_PushCharToFrame(data);
		//overflow - new data arrived before previous received.
	}

	if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) {
		__HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);

		data = (uint8_t) (USART1->DR & (uint8_t) 0xFF);
		GPS_PushCharToFrame(data);
	}

//	if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) != RESET) {
//		__HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TXE);
//		data = (uint8_t) (USART1->SR & (uint8_t) 0xFF);  //read status
//	}
}

//**end**//
