/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.h
  * @author  MCD Application Team
  * @version V2.2.0
  * @date    13-June-2014
  * @brief   Header for dfu_mal.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_TEMPLATE_H
#define __USBD_CDC_IF_TEMPLATE_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern USBD_CDC_ItfTypeDef  USBD_CDC_Template_fops;
struct FrameBuffer{
	uint8_t Msg[80];
	uint8_t Type;
	uint8_t Sender;
	uint8_t Size;
	enum {eStart = 0, eReciever, eSender, eType, eMsg, eDone} State;
	uint8_t isNew;
};

const uint8_t frameAddress_Pecet = 'P';
const uint8_t frameAddress_Cortex = 'C';

const uint8_t frameType_ConnectedWithGUI = 'S';
const uint8_t frameType_DisconnectedFromGUI = 'E';
const uint8_t frameType_StartIMUTimerUpdate = 's';
const uint8_t frameType_StopIMUTimerUpdate = 'e';

const uint8_t frameType_DataRequest = 'R';
const uint8_t frameType_AllDataRequest = 'D';
const uint8_t frameType_DataGatherRequest = 'G';
const uint8_t frameType_MahonyOrientationRequest = 'M';

const uint8_t frameType_Calibrate = 'C';
const uint8_t frameType_SendingTimeCheck = 'T';
const uint8_t frameType_FunctionExecutionTimeCheck = 'I';
const uint8_t frameType_Ping = 'P';
const uint8_t frameType_FunctionTest = 'F';

const uint8_t frameType_Log = 'L';
const uint8_t frameType_Error = 'E';


extern struct FrameBuffer s_RxFrameBuffer;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, uint16_t size );
int VCP_writeStringFrame(const char address, const char frameType, const void *pMsg);
int VCP_writeBinaryFrame(const char address, const void *pFrameType, const uint8_t typeSize, const void *pMsg, uint16_t msgSize);
int VCP_StringWrite(const char *pBuffer);
int VCP_Flush(void);
int VCP_isWriteComplete(void);

#endif /* __USBD_CDC_IF_TEMPLATE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
