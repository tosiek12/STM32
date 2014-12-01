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

//#include "Accelerometer/stm32f4_discovery_lis3dsh.h"
//#include "Accelerometer/accelerometer.h"

extern "C" {
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>

USBD_HandleTypeDef USBD_Device;
}

void SystemClock_Config(void);

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
 * TIM2 -> SpeedTester
 * TIM3 -> GPIO
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

	pwm.PWMInit();
	pwm.startPwmChannel(TIM_CHANNEL_1);
	//pwm.setChannelRawValue(1, 1000);

	while (1) {
		buttons.mainBegginingUpdate();
		numberOfChars = VCP_read(buf, 10);
		if (numberOfChars >= 1) {
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
				*cnt = strtol((char *) buf + 1, (char **) &(pEnd), 10);
				numberOfChars = sprintf((char *) buf, "\nX:\n%lu\n", *cnt);
				VCP_write(buf, numberOfChars);
				break;
			case 'T':
				if (*cnt > 1) {
					//*cnt = atol((char *) buf + 1);
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
		if (imu10DOF.getShowDataTriger() == 1) {
			//imu10DOF.showAnglesKalman(nokiaLCD);
			//imu10DOF.showMeasurment(nokiaLCD);
			imu10DOF.clearShowDataTriger();
		}

		if (imu10DOF.sendViaVirtualCom()) {
		}

		if (imu10DOF.isDataGatheringComplete()) {
			imu10DOF.sendGatheredDataViaVCOM();
		}

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

void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
	RCC_CLOCKTYPE_HCLK |
	RCC_CLOCKTYPE_PCLK1 |
	RCC_CLOCKTYPE_PCLK2);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler() {

	while (1) {
	}
}
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
