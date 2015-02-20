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
/*
 * [0] - imu connection [1 = ok, error, error when gathered]
 * [1] - gps connection
 */
volatile uint8_t flags[10] = {};
static uint8_t message[50];


///////////
#include "SD/spi_sd.h"
#include "SD/ff.h"
#include "SD/tm_stm32f4_fatfs.h"
////////////

int main() {
	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	Delay::initialize();	//Create, and initialize
	NokiaLCD nokiaLCD;	//Create, and initialize
	GPIO buttons;
	//buttons.InitButtons();
	//GPS_Init();

	//imu10DOF.initialize();
	//imu10DOF.startTimerUpdate();

	//imitate connection state
	imu10DOF.setConnected();
	//speedTester.tic();

	//pwm.PWMInit();
	//pwm.startPwmChannel(TIM_CHANNEL_1);
	//pwm.setChannelRawValue(1, 1000);
	//testBinaryCommunication();

	//testSD();

	SPI_SD_Init();
	testCreatingFiles();

	testCreatingFiles2();

	while (1) {
		buttons.mainBegginingUpdate();
		//GPS_Send();
		//Sprawdz czy trzeba i wykonaj akcje:
		//imu10DOF.sendViaVirtualCom();
		flags[0] = 1;
		if (imu10DOF.isDataGatheringComplete()) {
			imu10DOF.sendGatheredDataViaVCOM();
		}

		VCP_StartWriteAndWaitTillEnd();

		//Obsluga przyciskow:
		if (buttons.getButtonState(0) == GPIO::longPush) {
			nokiaLCD.WriteTextXY((char*) "longPush const", 0, 0);
		}
		if (buttons.getButtonStateChange(1, GPIO::shortPush)) {
		}
		if (buttons.getButtonState(0) == GPIO::longPush) {
		}
		buttons.mainEndUpdate();
	}
}
extern "C" {
void doFrameAction(void) {
	//Zmienne do ramki
	char buf[100] = { 0 };
	//inne
	uint32_t cnt[5] = { 0 };
	uint16_t val;
	uint8_t * pEnd;
	uint32_t numberOfChars;

	switch (s_RxFrameBuffer.Type) {
	case 'S':
		imu10DOF.setConnected();
		speedTester.tic();
		break;
	case 'E':
		imu10DOF.setDisconnected();
		speedTester.toc();
		break;
	case 'R':
		imu10DOF.setRequestOfData();
		imu10DOF.sendAngleViaVirtualCom();
		break;
	case 'C':
		imu10DOF.stopTimerUpdate();
		imu10DOF.calibrateGyroAndAccStationary();
//				imu10DOF.initialize();
		imu10DOF.startTimerUpdate();
		break;
	case 'X':
		break;
	case 'M':
		imu10DOF.sendMahonyViaVirtualCom();
		break;
	case 'T':
		pEnd= s_RxFrameBuffer.Msg + s_RxFrameBuffer.Size;
		val = strtol((char *) &s_RxFrameBuffer.Msg, (char **) &pEnd, 10);
		val = speedTester.testTimeOfSending(val);
		numberOfChars = sprintf((char *) buf, "\ntime:\n%lu\n", val);
		VCP_write(buf, numberOfChars);
		memset(buf, 0, 50);
		break;
	case 'I':
		speedTester.tic();
		imu10DOF.timerAction();
		cnt[0] = speedTester.delta();
		imu10DOF.timerAction();
		cnt[1] = speedTester.delta();
		imu10DOF.timerAction();
		cnt[2] = speedTester.delta();
		imu10DOF.timerAction();
		cnt[3] = speedTester.toc();

		numberOfChars = sprintf((char *) buf, "\ntime:\n%lu\n%lu\n%lu\n%lu\n", cnt[0],
				cnt[1], cnt[2], cnt[3]);
		VCP_write(buf, numberOfChars);
		break;
	case 'Z':
		trigger = 1;
		VCP_write("Connected\n", 10);
		break;
	case 'G':
		pEnd= s_RxFrameBuffer.Msg + s_RxFrameBuffer.Size;
		val = strtol((char *) &s_RxFrameBuffer.Msg, (char **) &pEnd, 10);
		imu10DOF.requestDataGathering(val);
		break;
	case 'A':
		VCP_StringWrite("Start: Testing SD-Card\n");
		//testBinaryCommunication();
		VCP_StringWrite("End: Testing SD-Card\n");
		break;
	default:
		break;
	}
	memset(buf, 0, 100);
}
}
void Error_Handler(void) {

	while (1) {
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
