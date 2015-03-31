//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "main.h"
#include "Delay/Delay.h"
#include "NokiaLCD/nokiaLCD.h"
#include "GPIO/gpio.h"
#include "10DOF/IMU.h"
#include "SpeedTester/speedTester.h"
#include "PWM/pwm.h"
#include "GPS/gps.h"
#include "SD/sdCardLogger.h"
//#include "Accelerometer/stm32f4_discovery_lis3dsh.h"
//#include "Accelerometer/accelerometer.h"

extern "C" {
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>

USBD_HandleTypeDef USBD_Device;
}

// Definitions visible only within this translation unit.
namespace {
// ----- Timing definitions -------------------------------------------------
constexpr uint32_t BLINK_ON_TICKS = 1000;
}

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"
//#pragma GCC diagnostic ignored "-Wmissing-declarations"
//#pragma GCC diagnostic ignored "-Wreturn-type"

/*
 * Using:
 * TIM1 -> PWM_generator
 * TIM3 -> IMU
 * TIM4 -> GPIO
 * TIM5 -> SpeedTester
 * SYSTIC -> GPIO,I2C,SPI(SDCard)
 */
volatile uint8_t trigger = 0;

/* Flags - inform about state of various process.
 * [0] - imu connection [1 = ok, error, error when gathered]
 * [1] - gps connection
 */
volatile uint8_t flags[10] = {};
/*
 *
 */
volatile uint8_t flagsComunicationInterface[10] = {};
volatile uint8_t flagsHardware[10] = {};

/* Global semaphores */
volatile uint8_t semaphore_timerInterrupt;


extern "C" {
static void doFrameAction(void) {
	//Zmienne do ramki
	char buf[100] = { 0 };
	//inne
	uint32_t cnt[5] = { 0 };
	uint16_t val;
	uint32_t numberOfChars;
	int16_t i = 0;
	int16_t numberOfSendChars = -1;

	FATFS FatFs;
	FIL fil;
	FRESULT state = FR_INT_ERR;

	if(s_RxFrameBuffer.isNew == 0) {
		return;
	} else {
		s_RxFrameBuffer.isNew = 0;
	}

	switch (s_RxFrameBuffer.Type) {
	case frameType_ConnectedWithGUI:
		flagsComunicationInterface[0] = f_connectedWithClient;
		strncpy((char *) buf,"IMU.txt",100);
		sdCardLogger.openFileForIMU(buf);
		imu10DOF.setConnected();
		break;
	case frameType_DisconnectedFromGUI:
		imu10DOF.stopTimerUpdate();
		imu10DOF.setDisconnected();
		flagsComunicationInterface[0] = f_connectedWithPC;
		sdCardLogger.closeFileForIMU();
		break;
	case frameType_StartIMUTimerUpdate:
		imu10DOF.startTimerUpdate();
		break;
	case frameType_DataRequest:
		imu10DOF.setRequestOfData();
		imu10DOF.sendAngleViaVirtualCom();
		break;
	case frameType_Calibrate:
		imu10DOF.stopTimerUpdate();
		imu10DOF.calibrateGyroAndAccStationary();
		imu10DOF.startTimerUpdate();
		break;
	case frameType_MahonyOrientationRequest:
		imu10DOF.sendMahonyViaVirtualCom();
		break;
	case frameType_SendingTimeCheck:
		s_RxFrameBuffer.Msg[s_RxFrameBuffer.Size] = '\0';
		val = atol((char *) &s_RxFrameBuffer.Msg);
		val = speedTester.testTimeOfSending(val);
		numberOfChars = sprintf((char *) buf, "time:%u\n", val);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_SendingTimeCheck, buf);
		memset(buf, 0, 50);
		break;
	case frameType_FunctionExecutionTimeCheck:
		speedTester.tic();
		imu10DOF.timerAction();
		cnt[0] = speedTester.toc();
		numberOfChars = sprintf((char *) buf, "Time: %lu\n", cnt[0]);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_FunctionExecutionTimeCheck, buf);
		break;
	case frameType_Ping:
		trigger = 1;
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Ping, "Connected\n");
		break;
	case 'G':
		s_RxFrameBuffer.Msg[s_RxFrameBuffer.Size] = '\0';
		val = atol((char *) &s_RxFrameBuffer.Msg);
		imu10DOF.requestDataGathering(val);
		break;
	case frameType_FunctionTest:
		if(s_RxFrameBuffer.Msg[0] == 'O') {
			strncpy((char *) buf,"IMU.txt",100);
			sdCardLogger.openFileForIMU(buf);
		}else if(s_RxFrameBuffer.Msg[0] == 'S') {
			sdCardLogger.writeStringForIMU((char *) s_RxFrameBuffer.Msg);
		}else if(s_RxFrameBuffer.Msg[0] == 'C') {
			sdCardLogger.closeFileForIMU();
		}
		break;
	default:
		numberOfChars = sprintf((char *) buf, "%s(%c))", "FrameTypeNotFound",s_RxFrameBuffer.Type);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Error,"FrameTypeNotFound");
		break;
	}
	memset(buf, 0, 100);
}
}
///////////
#include "SD/spi_sd.h"
#include "SD/ff.h"
////////////

int main() {
	/* Initialize Systick, */
	Delay::initialize();
	//NokiaLCD nokiaLCD;	//Create, and initialize
	GPIO buttons;
	//buttons.InitButtons();

	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	SPI_SD_Init();

	GPS_Init();

	imu10DOF.initialize();
	//speedTester.tic();

	//pwm.PWMInit();
	//pwm.startPwmChannel(TIM_CHANNEL_1);
	//pwm.setChannelRawValue(1, 1000);

	while (1) {
		buttons.mainBegginingUpdate();
		//GPS_Send();

		if (imu10DOF.isDataGatheringComplete()) {
			imu10DOF.sendGatheredDataViaVCOM();
		}

		doFrameAction();

		//Obsluga przyciskow:
		if (buttons.getButtonState(0) == GPIO::longPush) {
			//nokiaLCD.WriteTextXY((char*) "longPush const", 0, 0);
		}
		if (buttons.getButtonStateChange(1, GPIO::shortPush)) {
		}
		if (buttons.getButtonState(0) == GPIO::longPush) {
		}
		buttons.mainEndUpdate();
	}
}

void Error_Handler(void) {

	while (1) {
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
