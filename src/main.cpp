//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdint.h>
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
 * Used hardware elements:
 * TIM1 -> PWM_generator
 * TIM3 -> IMU
 * TIM4 -> GPIO
 * TIM5 -> SpeedTester
 * SYSTIC -> GPIO, I2C, SPI(SDCard)
 */

/* Global variables: */

/* Flags - inform about state of various process.
 * [0] - imu connection [1 = ok, error, error when gathered]
 * [1] - gps connection
 */
volatile uint8_t flagsComunicationInterface[10] = {};
volatile uint8_t flagsHardware[10] = {};

/* Semaphores */
volatile uint8_t semaphore_timerInterrupt;

/* End of Global variables */
extern "C" {
static void doFrameAction(void) {
	//Zmienne do ramki
	static char buf[200] = { 0 };

	//inne
	uint32_t cnt[5] = { 0 };
	uint16_t value;

	if(s_RxFrameBuffer.isNew == 0) {
		return;
	} else {
		s_RxFrameBuffer.isNew = 0;
	}

	switch (s_RxFrameBuffer.Type) {
	case frameType_ConnectedWithGUI:
		flagsComunicationInterface[f_interface_USB] = f_connectedWithClient;
		io_module.turnLedOn(GPIO::LED3_PIN, 1);
		sdCardLogger.openFile(SdCardLogger::debug, "debug.txt");
		imu10DOF.setConnected();
		break;
	case frameType_DisconnectedFromGUI:
		imu10DOF.stopTimerUpdate();
		imu10DOF.setDisconnected();
		flagsComunicationInterface[f_interface_USB] = f_connectedWithPC;
		io_module.turnLedOn(GPIO::LED3_PIN, 0);
		sdCardLogger.closeFile(SdCardLogger::measurement);
		sdCardLogger.closeFile(SdCardLogger::debug);
		break;
	case frameType_StartIMUTimerUpdate:
		strncpy((char *) buf,"imu.csv",100);
		sdCardLogger.openFile(SdCardLogger::measurement, buf);
		imu10DOF.startTimerUpdate();
		io_module.turnLedOn(GPIO::LED4_PIN, 1);
		break;
	case frameType_StopIMUTimerUpdate:
		imu10DOF.stopTimerUpdate();
		sdCardLogger.closeFile(SdCardLogger::measurement);
		io_module.turnLedOn(GPIO::LED4_PIN, 0);
		break;
	case frameType_DataRequest:
		//imu10DOF.setRequestOfData();
		//imu10DOF.sendAngleViaVirtualCom();
		break;
	case frameType_AllDataRequest:
		imu10DOF.prepareDataFrame((uint8_t *) buf,200);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_AllDataRequest, buf);
		break;
	case frameType_Calibrate:
		imu10DOF.stopTimerUpdate();
		imu10DOF.calibrateGyroAndAccStationary();
		imu10DOF.startTimerUpdate();
		break;
	case frameType_SendingTimeCheck:
		s_RxFrameBuffer.Msg[s_RxFrameBuffer.Size] = '\0';
		value = atol((char *) &s_RxFrameBuffer.Msg);
		value = speedTester.testTimeOfSending(value);
		sprintf((char *) buf, "time:%u\n", value);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_SendingTimeCheck, buf);
		memset(buf, 0, 50);
		break;
	case frameType_FunctionExecutionTimeCheck:
		speedTester.tic();
		imu10DOF.logDataOnSD();
		cnt[0] = speedTester.toc();
		sprintf((char *) buf, "Time: %lu\n", cnt[0]);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_FunctionExecutionTimeCheck, buf);
		break;
	case frameType_Ping:
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Ping, "Connected\n");
		break;
	case 'G':
		s_RxFrameBuffer.Msg[s_RxFrameBuffer.Size] = '\0';
		value = atol((char *) &s_RxFrameBuffer.Msg);
		imu10DOF.requestDataGathering(value);
		break;
	case frameType_FunctionTest:
		if(s_RxFrameBuffer.Msg[0] == 'O') {
			strncpy((char *) buf,"IMU.txt",100);
			sdCardLogger.openFile(SdCardLogger::measurement,buf);
		}else if(s_RxFrameBuffer.Msg[0] == 'S') {
			sdCardLogger.writeStringForIMU((char *) s_RxFrameBuffer.Msg);
		}else if(s_RxFrameBuffer.Msg[0] == 'C') {
			sdCardLogger.closeFile(SdCardLogger::measurement);
		}
		break;
	default:
		sprintf((char *) buf, "%s(%c))", "FrameTypeNotFound", s_RxFrameBuffer.Type);
		VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, buf);
		break;
	}
	memset(buf, 0, 100);
}
}

int main() {
	/* Initialize Systick, */
	Delay::initialize();
	//NokiaLCD nokiaLCD;	//Create, and initialize
	io_module.InitButtons();
	io_module.InitLeds();
	sdCardLogger.initialize();
	//dupa in main branch
	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	GPS_Init();
	imu10DOF.initialize();

	//pwm.PWMInit();
	//pwm.startPwmChannel(TIM_CHANNEL_1);
	//pwm.setChannelRawValue(1, 1000);

	while (1) {
		io_module.mainBegginingUpdate();

		doFrameAction();

		//Obsluga przyciskow:
		if (io_module.getButtonState(0) == GPIO::longPush) {
			//nokiaLCD.WriteTextXY((char*) "longPush const", 0, 0);
		}
		if (io_module.getButtonStateChange(1, GPIO::shortPush)) {
		}
		if (io_module.getButtonState(0) == GPIO::longPush) {
		}
		io_module.mainEndUpdate();
	}
}
void Error_Handler(void) {
	while (1) {
		io_module.turnLedOn(GPIO::LED3_PIN,1);
		Delay::delay_ms(10);
		io_module.turnLedOn(GPIO::LED3_PIN,0);
		Delay::delay_ms(10);
	}
}

void Error_Handler(const char *pbuf) {
	VCP_writeStringFrame(frameAddress_Pecet, frameType_Error, pbuf);
	while (1) {
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
