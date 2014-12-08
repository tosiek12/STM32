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
#include "SDCard/tm_stm32f4_fatfs.h"
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

int main() {
	USBD_Init(&USBD_Device, &VCP_Desc, 0);
	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	Delay::initialize();	//Create, and initialize
	NokiaLCD nokiaLCD;	//Create, and initialize
	GPIO buttons;
	buttons.InitButtons();

	imu10DOF.initialize();
	imu10DOF.startTimerUpdate();

	uint8_t buf[50] = { 0 };
	uint32_t cnt[5] = { 0 };
	uint32_t numberOfChars;
	uint8_t * pEnd;

	//imitate connection state
	imu10DOF.setConnected();
	speedTester.tic();

	//pwm.PWMInit();
	//pwm.startPwmChannel(TIM_CHANNEL_1);
	//pwm.setChannelRawValue(1, 1000);

	GPS_Init();


	while (1) {
		buttons.mainBegginingUpdate();

		//GPS_Send();
		//Format ramki: [$kod,wartosc*]
		if ((numberOfChars = VCP_read(buf, 10)) >= 1) {
			switch (buf[0]) {
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
				break;
			case 'C':
				imu10DOF.calibrateAllSensors();
				break;
			case 'X':
				break;
			case 'T':
				if (*cnt > 1) {
					*cnt = strtol((char *) buf + 1, (char **) &(pEnd), 10);
					*cnt = speedTester.testTimeOfSending(*cnt);
					numberOfChars = sprintf((char *) buf, "\ntime:\n%lu\n", *cnt);
					VCP_write(buf, numberOfChars);
					memset(buf, 0, 50);
				}
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
				VCP_write("Connected", 9);
				break;
			case 'G':
				if (numberOfChars > 1) {
					*cnt = atol((char *) buf + 1);
					imu10DOF.requestDataGathering(*cnt);
				}
				break;
			default:
				break;
			}
			memset(buf, 0, 15);
		}

		//Sprawdz cy trzeba i wykonaj akcje:
		imu10DOF.sendViaVirtualCom();
		if (imu10DOF.isDataGatheringComplete()) {
			imu10DOF.sendGatheredDataViaVCOM();
		}

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

void Error_Handler() {

	while (1) {
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
