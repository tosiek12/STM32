/**
 ******************************************************************************
 * @file    usbd_cdc_if_template.c
 * @author  MCD Application Team
 * @version V2.2.0
 * @date    13-June-2014
 * @brief   Generic media access Layer.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if_template.h"
#include "main.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_CDC 
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_CDC_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_CDC_Private_Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_CDC_Private_FunctionPrototypes
 * @{
 */

static int8_t TEMPLATE_Init(void);
static int8_t TEMPLATE_DeInit(void);
static int8_t TEMPLATE_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t TEMPLATE_Receive(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_Template_fops = { TEMPLATE_Init, TEMPLATE_DeInit, TEMPLATE_Control,
		TEMPLATE_Receive };

USBD_CDC_LineCodingTypeDef linecoding = { 460800, /* baud rate*/	//115200
		0x00, /* stop bits-1*/
		0x00, /* parity - none*/
		0x08 /* nb. of bits 8*/
};

/* Private functions ---------------------------------------------------------*/
extern USBD_HandleTypeDef USBD_Device;

static struct {
	uint8_t Buffer[CDC_DATA_HS_IN_PACKET_SIZE];
	uint16_t Position;
	uint16_t Size;
	uint8_t ReadDone;
} s_RxBuffer;

static struct {
	uint8_t Buffer[CDC_DATA_HS_MAX_PACKET_SIZE];
	uint16_t Position;
	uint16_t SizeOfDataToWrite;
	uint8_t WriteDone;
} s_TxBuffer;

struct FrameBuffer s_RxFrameBuffer;

/**
 * @brief  TEMPLATE_Init
 *         Initializes the CDC media low layer
 * @param  None
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t TEMPLATE_Init(void) {

	USBD_CDC_SetRxBuffer(&USBD_Device, s_RxBuffer.Buffer);
	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *) s_TxBuffer.Buffer, 0);
	flagsComunicationInterface[f_interface_USB] = f_connectedWithPC;
	s_RxFrameBuffer.Size = 0;
	s_RxFrameBuffer.State = eStart;
	return (USBD_OK);
}

/**
 * @brief  TEMPLATE_DeInit
 *         DeInitializes the CDC media low layer
 * @param  None
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t TEMPLATE_DeInit(void) {
	flagsComunicationInterface[f_interface_USB] = f_configured;
	return (USBD_OK);
}

/**
 * @brief  TEMPLATE_Control
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t TEMPLATE_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
	switch (cmd) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Add your code here */
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Add your code here */
		break;

	case CDC_SET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_GET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_SET_LINE_CODING:
		linecoding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) |\
 (pbuf[2] << 16)
				| (pbuf[3] << 24));
		linecoding.format = pbuf[4];
		linecoding.paritytype = pbuf[5];
		linecoding.datatype = pbuf[6];

		/* Add your code here */
		break;

	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t) (linecoding.bitrate);
		pbuf[1] = (uint8_t) (linecoding.bitrate >> 8);
		pbuf[2] = (uint8_t) (linecoding.bitrate >> 16);
		pbuf[3] = (uint8_t) (linecoding.bitrate >> 24);
		pbuf[4] = linecoding.format;
		pbuf[5] = linecoding.paritytype;
		pbuf[6] = linecoding.datatype;

		/* Add your code here */

		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/* Add your code here */

		//Rozlaczenie od lini
		break;

	case CDC_SEND_BREAK:
		/* Add your code here */
		break;

	default:
		break;
	}

	return (0);
}

/**
 * @brief  TEMPLATE_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on CDC interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t TEMPLATE_Receive(uint8_t* Buf, uint32_t *Len) {
	//oblsuga dekodowania ramki
	uint8_t currentChar;
	for(uint16_t it = 0; it<*Len; it++) {
		currentChar = *(Buf+it);

		if(currentChar == '$') {
			s_RxFrameBuffer.State = eReciever;
			continue;
		}
		if(currentChar == '\r' || currentChar == '\n') {
			//error;
			s_RxFrameBuffer.State = eStart;
			continue;
		}

		switch(s_RxFrameBuffer.State) {
		case eStart:
			if(currentChar == '$') {
				s_RxFrameBuffer.State = eReciever;
			}
			break;
		case eReciever:
			if(currentChar != frameAddress_Cortex) {
				s_RxFrameBuffer.State = eStart;	//it isn't for this device
			} else {
				s_RxFrameBuffer.State = eSender;
			}
			break;
		case eSender:
			//for future purpose - do nothing now.
			s_RxFrameBuffer.Sender = currentChar;
			s_RxFrameBuffer.State = eType;
			break;
		case eType:
			s_RxFrameBuffer.Type = currentChar;
			s_RxFrameBuffer.State = eMsg;
			s_RxFrameBuffer.Size = 0;
			break;
		case eMsg:
			if (currentChar == '*' ) {
				s_RxFrameBuffer.State = eDone;
				//obsluga odebranej ramki ramki - gdy sie dzieje w przerwaniu to TxState == 1 i nie wykonuje akcji!!!
				if(s_RxFrameBuffer.isNew == 1) {
					//errror - nie obsluzono poprzedniej ramki ;(
				}
				s_RxFrameBuffer.isNew = 1;
				s_RxFrameBuffer.State = eStart;
			} else if (s_RxFrameBuffer.Size == 80) {
				//error - za dluga ramka
				s_RxFrameBuffer.State = eStart;
			} else {
				*(s_RxFrameBuffer.Msg + s_RxFrameBuffer.Size) = currentChar;
				++s_RxFrameBuffer.Size;
			}
			break;
		}
	}

	//Wyzwzol odbieranie nastepnych danych.
	USBD_CDC_ReceivePacket(&USBD_Device);
	return (0);
}

//int VCP_read(void *pBuffer, int size) {
//	return 0 ;
//
//	if (!s_RxBuffer.ReadDone)
//		return 0;
//
//	int remaining = s_RxBuffer.Size - s_RxBuffer.Position;
//	int todo = MIN(remaining, size);
//	if (todo <= 0)
//		return 0;
//
//	memcpy(pBuffer, s_RxBuffer.Buffer + s_RxBuffer.Position, todo);
//	s_RxBuffer.Position += todo;
//	if (s_RxBuffer.Position >= s_RxBuffer.Size) {
//		s_RxBuffer.ReadDone = 0;
//		USBD_CDC_ReceivePacket(&USBD_Device);
//	}
//
//	return todo;
//}

//Add data to send buffer. If size is bigger than output buffer send max size.
int VCP_write(const void *pBuffer, uint16_t size ) {

	if(flagsComunicationInterface[f_interface_USB] != f_connectedWithClient) {
		return 0;
	}

	int todo = 0;
	USBD_CDC_HandleTypeDef *pCDC = (USBD_CDC_HandleTypeDef *) USBD_Device.pClassData;
	if (pCDC == NULL) {	//device not connected
		return 0;
	}
	if(!semaphore_timerInterrupt) {	//If not in interrupt block and wait till end.
		while (pCDC->TxState) {
		} //Wait for previous transfer
	}

	todo = MIN((CDC_DATA_HS_OUT_PACKET_SIZE - pCDC->TxLength - 1), size);
	memcpy((s_TxBuffer.Buffer + pCDC->TxLength), pBuffer, todo);
	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *) s_TxBuffer.Buffer, (pCDC->TxLength + todo));

	if (USBD_CDC_TransmitPacket(&USBD_Device) != USBD_OK) {	//MOJE
		return 0;
	}	//Transmit first packet, rest will follow in interrupts.

	return todo;
}
#define FRAMEBUFFERSIZE 400
static uint8_t frameBuffer[FRAMEBUFFERSIZE];
static uint8_t  numberOfChars;
int VCP_writeStringFrame(const char address,const char frameType, const void *pMsg) {
	numberOfChars = snprintf((char *) frameBuffer, FRAMEBUFFERSIZE, "$%cC%c%s*",address, frameType, pMsg);
	return VCP_write(frameBuffer, numberOfChars);
}
int VCP_writeBinaryFrame(const char address, const void *pFrameType, const uint8_t typeSize, const void *pMsg, uint16_t msgSize) {
	numberOfChars = snprintf((char *) frameBuffer, FRAMEBUFFERSIZE, "$%cC%s",address, pFrameType);
	numberOfChars = VCP_write(frameBuffer, numberOfChars);
	numberOfChars += VCP_write(pMsg, msgSize);
	numberOfChars += VCP_write("*", 1);
	return numberOfChars;
}


int VCP_StringWrite(const char *pBuffer) {
	return VCP_write(pBuffer, strlen(pBuffer));
}

int VCP_Flush(void) {
	if(flagsComunicationInterface[f_interface_USB] != f_connectedWithClient) {
		return 0;
	}
	USBD_CDC_HandleTypeDef *pCDC = (USBD_CDC_HandleTypeDef *) USBD_Device.pClassData;

		if (pCDC == NULL) {	//device not connected.
			return 0;
		}
		if( pCDC->TxLength != 0) {
			if (USBD_CDC_TransmitPacket(&USBD_Device) != USBD_OK) {
					return 0;
				}
				while (pCDC->TxState) {
				} //Wait for previous transfer
				pCDC->TxLength = 0;
		}
		return 1;
}

int VCP_isWriteComplete(void) {
	USBD_CDC_HandleTypeDef *pCDC = (USBD_CDC_HandleTypeDef *) USBD_Device.pClassData;
	if (pCDC == NULL || pCDC->TxState == 1) {	//device not connected or previous transmition in proggres
		return 0;
	} else {
		return 1;
	}
}

void VCP_setTxBufferToZero(void) {
s_TxBuffer.SizeOfDataToWrite = 0;
}

int VCP_write_old(const void *pBuffer, int size) {
	if (size > CDC_DATA_HS_OUT_PACKET_SIZE) {
		int offset;
		for (offset = 0; offset < size; offset += CDC_DATA_HS_OUT_PACKET_SIZE) {
			int todo = MIN(CDC_DATA_HS_OUT_PACKET_SIZE, size - offset);
			int done = VCP_write(((char *) pBuffer) + offset, todo);
			if (done != todo)
				return offset + done;
		}

		return size;
	}

	USBD_CDC_HandleTypeDef *pCDC = (USBD_CDC_HandleTypeDef *) USBD_Device.pClassData;

	if (pCDC == NULL) {	//device not connected.
		return 0;
	}


	USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t *) s_TxBuffer.Buffer, size);
	if (USBD_CDC_TransmitPacket(&USBD_Device) != USBD_OK)
		return 0;

	while (pCDC->TxState) {
	} //Wait until transfer is done
	return size;
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

