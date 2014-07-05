//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "BlinkLed.h"

// ----------------------------------------------------------------------------

void BlinkLed::powerUp() {
	// Enable GPIO Peripheral clock
//	RCC->AHB1ENR |= BLINK_RCC_MASKx(BLINK_PORT_NUMBER);
	//BLINK_PORT_NUMBER == 3 - GPIOD
	__GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);
	// Start with led turned off
	turnOff();
}

// ----------------------------------------------------------------------------
