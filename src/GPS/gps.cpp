#include "GPS/gps.h"
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
void GPS_Send() {
	 if(isNewFrame) {
		 pFrame[frameSize] = '\n';
		 VCP_write(pFrame, frameSize+1);
		 isNewFrame = 0;
	 }
 }

void GPS_PushCharToFrame(volatile uint8_t recievedChar) {
	if (charToSkip > 0) {
		--charToSkip;	//skip check Sum
		return;
	}

	if (recievedChar == '$') {
		memset(pTempFrame, 0, 80);
		pTempFrame[0] = recievedChar;
		charsInFrame = 1;
	} else if (pTempFrame[0] == '$') {
		if (recievedChar == '*') {
			uint8_t *temp;
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

			//interpretuj ramke
			if (!strncmp((char *) pFrame, "$GPGGA", 6)) {
				//printf("nowa: %s\n", pFrame);
			}
//			NMEA_Parse(pFrame, frameSize);
//			gpsdata;
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

struct gpsData_t gpsdata;

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

//Potrzeba:
//GGA - Global Positioning System Fix Data
//GSA - GNSS DOP and Active Satellites
//RMC - Recommended Minimum Specific GNSS Data
//VTG - Course Over Ground and Ground Speed
void GPS_Parse(uint8_t *buf, uint8_t len)
{
	char *p;
	int32_t tmp;
	p = (char *)buf;

	if (!strncmp(p, "$GPGGA", 6))
	{
		//123519 – Aktualność danych - 12:35 : 19 UTC,
		p += 7;
		gpsdata.hour = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.min = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.sec = (p[4] - '0') * 10 + p[5] - '0';


		//4807.038, N – szerokość geograficzna(latitude) - 48 deg 07.038' N,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		gpsdata.lat = tmp;

		//01131.000, E – długość geograficzna(longitude) - 11 deg 31.000' E,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		gpsdata.lon = tmp;

		//1 – jakość pomiaru
		p = strchr(p, ',') + 1;
		gpsdata.valid = (p[0] - '0') ? 1 : 0;

		//08 – ilość śledzonych satelitów,
		p = strchr(p, ',') + 1;
		gpsdata.sats = (p[0] - '0') * 10 + p[1] - '0';

		//0.9 – horyzontalna dokładność pozycji(HDOP) (opisana dalej),
		p = strchr(p, ',') + 1;
		gpsdata.hdop = GPS_atoi(p);

		//545.4, M – wysokość w metrach nad poziom morza,
		p = strchr(p, ',') + 1;
		gpsdata.alt = GPS_atoi(p);
	}
	else if (!strncmp(p, "$GPRMC", 6)) {

		//123519 – Aktualność danych - 12:35:19 UTC,
		p += 7;
		gpsdata.hour = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.min = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.sec = (p[4] - '0') * 10 + p[5] - '0';

		// A – status(A – aktywny; V – nieaktywny),
		p = strchr(p, ',') + 1;
		gpsdata.valid = (p[0] == 'A') ? 1 : 0;

		//4807.038,N – szerokość geograficzna (latitude) - 48 deg 07.038' N,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'S')
			tmp = -tmp;
		gpsdata.lat = tmp;

		//01131.000, E – długość geograficzna(longitude) - 11 deg 31.000' E,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'W')
			tmp = -tmp;
		gpsdata.lon = tmp;

		//022.4 – prędkość obiektu(liczona w węzłach),
		p = strchr(p, ',') + 1;
		gpsdata.speed = GPS_atoi(p);

		//084.4 – kąt śledzenia / poruszania się obiektu(w stopniach)
		p = strchr(p, ',') + 1;
		gpsdata.heading = GPS_atoi(p);

		//230394 – data(23 marca 1994),
		p = strchr(p, ',') + 1;
		gpsdata.day = (p[0] - '0') * 10 + p[1] - '0';
		gpsdata.month = (p[2] - '0') * 10 + p[3] - '0';
		gpsdata.year = (p[4] - '0') * 10 + p[5] - '0';
	}
	else if (!strncmp(p, "$GPGSA", 6)) {

		//A – automatyczny wybór pozycji (2D lub 3D) /M – manualny/,
		p += 7;
		//mode = p[0];

		// 3 – pozycja 3D. Możliwe wartości to:
		p = strchr(p, ',') + 1;
		//position3D = (p[0] - '0');

		//04,05... – Numery satelitów wykorzystane do wyznaczenia pozycji (miejsce dla 12 satelitów),
		for (uint8_t it = 0; it < 11; it++) {
			p = strchr(p, ',') + 1;
			//numer[it] = (p[0] - '0') * 10 + p[1] - '0';
		}

		//2.5 – DOP (dilution of precision) – precyzja wyznaczonej pozycji,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);

		//1.3 – HDOP (horizontal dilution of precision) – horyzontalna precyzja,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);

		//2.1 – VDOP (vertical dilution of precision) – precyzja wertykalna,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
	}
	else if (!strncmp(p, "$GPVTG", 6)) {
		//054.7,T – ścieżka poruszania się (w stopniach),
		p += 7;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'T')
			//gpsdata.speed = tmp;

		//034.4,M – ścieżka poruszania się (na podstawię współrzędnych magnetycznych – w stopniach),
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'M')
			//gpsdata.speed = tmp;

		//005.5,N – prędkość w węzłach,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'N')
			//gpsdata.speed = tmp;

		//010.2,K – prędkość w km/h,
		p = strchr(p, ',') + 1;
		tmp = GPS_atoi(p);
		p = strchr(p, ',') + 1;
		if (p[0] == 'K')
			gpsdata.speed = tmp;
	}
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

	HAL_UART_Transmit(&UartHandle,(unsigned char *)"", 5, 4000);

	/* Peripheral interrupt init*/
	HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
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
		VCP_write("OVF", 3);
	}

	if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) {
		__HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);

		data = (uint8_t) (USART1->DR & (uint8_t) 0xFF);
		GPS_PushCharToFrame(data);
	}
//
//	if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) != RESET) {
//		__HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TXE);
//		data = (uint8_t) (USART1->SR & (uint8_t) 0xFF);  //read status
//	}
}

//**end**//
