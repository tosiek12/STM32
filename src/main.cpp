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

//#include "Accelerometer/stm32f4_discovery_lis3dsh.h"
#include "Accelerometer/accelerometer.h"
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
	__SYSCFG_CLK_ENABLE();

	Delay::initialize();	//Create, and initialize
	NokiaLCD nokiaLCD;	//Create, and initialize
	BlinkLed blinkLed;
	GPIO buttons;

	buttons.InitButtons();

	nokiaLCD.Clear();
	nokiaLCD.WriteTextXY((char*) "TROLOLO", 0, 0);
	nokiaLCD.WriteBMP();
	Delay::delay_ms(1000);

	//InitAccelerometer();
	//acc_initialized = 1;

	IMU imu10DOF;	//
	imu10DOF.gyro.initialize();
	imu10DOF.accelerometer.initialize();
//	if (gyro.testConnection() == 0) {
//		while (1) {
//		}	//Error handler.
//	}

	uint8_t buf[10];

	// Perform all necessary initializations for the LED.
	blinkLed.powerUp();
	uint16_t c = 0;
	while (1) {

		buttons.mainBegginingUpdate();
		if (++c == 1000) {
			//Main_AccelerometerAction(&nokiaLCD);
			imu10DOF.accelerometer.test(nokiaLCD);
			imu10DOF.gyro.test(nokiaLCD);
			c = 0;
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

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
