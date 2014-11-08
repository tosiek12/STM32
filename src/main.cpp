//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

#include "Delay/Delay.h"
#include "BlinkLed.h"
#include "NokiaLCD/nokiaLCD.h"
#include "GPIO/gpio.h"
#include "10DOF/IMU.h"

#include "SDCard/tm_stm32f4_fatfs.h"

//#include "Accelerometer/stm32f4_discovery_lis3dsh.h"
//#include "Accelerometer/accelerometer.h"

extern "C" {
#include <usbd_core.h>
#include <usbd_cdc.h>
#include <usbd_cdc_if_template.h>
#include <usbd_desc.h>

USBD_HandleTypeDef USBD_Device;
}

static void SystemClock_Config(void) {
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


void testSD();
FRESULT TM_FATFS_DriveSize2(uint32_t* total, uint32_t* free) {
	FATFS *fs;
    DWORD fre_clust;
	FRESULT res;

    /* Get volume information and free clusters of drive */
    res = f_getfree("0:", &fre_clust, &fs);
    if (res != FR_OK) {
		return res;
	}

    /* Get total sectors and free sectors */
    *total = (fs->n_fatent - 2) * fs->csize / 2;
    *free = fre_clust * fs->csize / 2;

	/* Return OK */
	return FR_OK;
}
// ----------------------------------------------------------------------------
//
// STM32F4 led blink sample (trace via ITM).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the ITM output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).

// Definitions visible only within this translation unit.
namespace {
// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
constexpr uint32_t BLINK_ON_TICKS = 1000;
constexpr uint32_t BLINK_OFF_TICKS = 500;
}

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"



int main(int argc, char* argv[]) {
	HAL_Init();
	SystemClock_Config();
	USBD_Init(&USBD_Device, &VCP_Desc, 0);


	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_Template_fops);
	USBD_Start(&USBD_Device);

	Delay::initialize();	//Create, and initialize
	NokiaLCD nokiaLCD;	//Create, and initialize
	BlinkLed blinkLed;
	GPIO buttons;

	buttons.InitButtons();

	nokiaLCD.Clear();

	imu10DOF.initialize();


	uint8_t buf[10];

	// Perform all necessary initializations for the LED.
	blinkLed.powerUp();
	uint16_t counter = 0;
	Kalman test;

	testSD();

	while (1) {
		buttons.mainBegginingUpdate();

		if((VCP_read(buf,1) == 1)) {
			switch(buf[0]) {
			case 'S':
				imu10DOF.setConnected();
				break;
			case 'E':
				imu10DOF.setDisconnected();
				break;
			case 'R':
				imu10DOF.setRequestOfData();
				break;
			case 'C':
				imu10DOF.calibrateAllSensors();
				break;
			case 'T':
				imu10DOF.computeAngles();
				imu10DOF.kalmanStepAction();
				break;
			case 'Z':
				testSD();
				break;
			default:

				break;
			}
		}
		if(imu10DOF.getShowDataTriger() == 1) {
			//imu10DOF.showAnglesKalman(nokiaLCD);
			//imu10DOF.showMeasurment(nokiaLCD);
			imu10DOF.clearShowDataTriger();
		}

		 if(imu10DOF.sendViaVirtualCom()) {
			 if(++counter == 10000) {
				 counter = 0;
			 }
		 }

		if (buttons.getButtonState(0) == GPIO::longPush) {
			nokiaLCD.WriteTextXY((char*) "longPush const", 0, 0);
		}

		if (buttons.getButtonStateChange(1, GPIO::shortPush)) {
			blinkLed.turnOn();
			nokiaLCD.WriteTextXY((char*) "1", 5, 5);
			Delay::delay_ms(BLINK_ON_TICKS);
			blinkLed.turnOff();
			nokiaLCD.WriteTextXY((char*) "0", 5, 5);
			Delay::delay_ms(BLINK_OFF_TICKS);
		}
		if (buttons.getButtonState(0) == GPIO::longPush) {
		}

		buttons.mainEndUpdate();
	}
}

void testSD() {
	FATFS FatFs;
	//File object
	FIL fil;
	//Free and total space
	uint32_t total, free;
	volatile uint8_t state = 0;
	//Mount drive
	state = f_mount(&FatFs, "", 0);
	if (state == FR_OK) {
		//Mounted OK, turn on RED LED

		//Try to open file
		state = f_open(&fil, "1stfile", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
		if (state == FR_OK) {
			//File opened, turn off RED and turn on GREEN led

			//If we put more than 0 characters (everything OK)
			state = f_puts("First string in my file\n", &fil);
			if (state > 0) {
				state = TM_FATFS_DriveSize2(&total, &free) ;
				if (state == FR_OK) {
					//Data for drive size are valid
				}

				//Turn on both leds
			}

			//Close file, don't forget this!
			f_close(&fil);
		}

		//Try to open file
		if (f_open(&fil, "2ndfile.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
			//File opened, turn off RED and turn on GREEN led

			//If we put more than 0 characters (everything OK)
			if (f_puts("First string in my file\n", &fil) > 0) {
				if (TM_FATFS_DriveSize2(&total, &free) == FR_OK) {
					//Data for drive size are valid
				}

				//Turn on both leds
			}

			//Close file, don't forget this!
			f_close(&fil);
		}

		//Unmount drive, don't forget this!
		f_mount(0, "", 1);
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
