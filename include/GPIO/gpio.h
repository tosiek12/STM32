#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include "cmsis_device.h"
#include "stm32f4xx_hal.h"

class GPIO {
private:
#define __GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define BTN1_PORT GPIOB
#define BTN1_PIN GPIO_PIN_5
#define BTN1_PIN_Line EXTI_Line5

#define BTN2_PORT GPIOB
#define BTN2_PIN GPIO_PIN_7
#define BTN2_PIN_Line EXTI_Line7

#define BTN3_PORT GPIOB
#define BTN3_PIN GPIO_PIN_8
#define BTN3_PIN_Line EXTI_Line8

public:
	static const uint16_t LED3_PIN = GPIO_PIN_13;
	static const uint16_t LED4_PIN = GPIO_PIN_12;
	static const uint16_t LED5_PIN = GPIO_PIN_14;
	static const uint16_t LED6_PIN = GPIO_PIN_15;
	enum state {
		noPush = 0, shortPush = 1, mediumPush = 2, longPush = 3
	};
	GPIO() = default;

	/*
	 * Wykonywane co 1ms - musi byc skonfigurowany jakis timer.
	 * Robienie tego na SysTick z programowym licznikiem jest bez sensu...
	 */
	void SysTick_ButtonAction();

	state getButtonState(uint8_t number) {
		return (tempButtons[number]);
	}

	uint8_t getButtonStateChange(uint8_t number, state typeOfState) {
		return (tempButtons[number] == typeOfState
				&& oldButtons[number] != typeOfState);
	}

	inline void __attribute__((always_inline)) mainBegginingUpdate() {
		tempButtons[0] = buttons[0];	//Copy buttons states for current loop.
		tempButtons[1] = buttons[1];
		tempButtons[2] = buttons[2];
	}

	inline void __attribute__((always_inline)) mainEndUpdate() {
		oldButtons[0] = tempButtons[0];	//Copy old buttons states.
		oldButtons[1] = tempButtons[1];
		oldButtons[2] = tempButtons[2];
	}

	static void Timer_ButtonAction() {
		if (buttonsTime[0] > 0) {
			--buttonsTime[0];
			if (buttonsTime[0] == MEDIUM_TIME_IN_MS) {
				buttons[0] = mediumPush;
			} else if (buttonsTime[0] == 0) {
				buttons[0] = longPush;
			}
		}
		if (buttonsTime[1] > 0) {
			--buttonsTime[1];
			if (buttonsTime[1] == MEDIUM_TIME_IN_MS) {
				buttons[1] = mediumPush;
			} else if (buttonsTime[1] == 0) {
				buttons[1] = longPush;
			}
		}
		if (buttonsTime[2] > 0) {
			--buttonsTime[2];
			if (buttonsTime[2] == MEDIUM_TIME_IN_MS) {
				buttons[2] = mediumPush;
			} else if (buttonsTime[2] == 0) {
				buttons[2] = longPush;
			}
		}
	}

	static void EXTI_ButtonAction() {
		if (__HAL_GPIO_EXTI_GET_IT(BTN1_PIN_Line) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_FLAG(BTN1_PIN_Line);
			if ((BTN1_PORT->IDR & BTN1_PIN) == (uint32_t) RESET) {
				buttons[0] = shortPush;
				buttonsTime[0] = LONG_TIME_IN_MS;
			} else {
				buttons[0] = noPush;
				buttonsTime[0] = 0;
			}
		}

		if (__HAL_GPIO_EXTI_GET_IT(BTN2_PIN_Line) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_FLAG(BTN2_PIN_Line);
			if ((BTN2_PORT->IDR & BTN2_PIN) == (uint32_t) RESET) {
				buttons[1] = shortPush;
				buttonsTime[1] = LONG_TIME_IN_MS;
			} else {
				buttons[1] = noPush;
				buttonsTime[1] = 0;
			}
		}

		if (__HAL_GPIO_EXTI_GET_IT(BTN3_PIN_Line ) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_FLAG(BTN3_PIN_Line);
			if ((BTN3_PORT->IDR & BTN3_PIN) == (uint32_t) RESET) {
				buttons[2] = shortPush;
				buttonsTime[2] = LONG_TIME_IN_MS;
			} else {
				buttons[2] = noPush;
				buttonsTime[2] = 0;
			}
		}
	}


	void turnLedOn (uint16_t ledNumber, uint8_t _state) {
		if(_state) {
			GPIOD->BSRRL |= ledNumber;
		}  else {
			GPIOD->BSRRH |= ledNumber;
		}
	}
	/**
	 * Configure all leds as output,
	 * LED3, LED4 are possible as AF - for PWM from TIM4
	 */
	inline void __attribute__((always_inline)) InitLeds() {
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Periph clock enable */
		__GPIOD_CLK_ENABLE();	// The LEDs are on GPIOD

		GPIO_InitStructure.Pin = LED3_PIN | LED4_PIN | LED5_PIN | LED6_PIN;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	}

	/*
	 * Initialize PB5,PB6,PB7 as PuPd NOPULL input.
	 */
	inline void __attribute__((always_inline)) InitButtons() {
		__SYSCFG_CLK_ENABLE();

		InitTimer();

		GPIO_InitTypeDef GPIO_InitStructure;
		/* Periph clock enable */
		__GPIO_CLK_ENABLE();
		/* Configure pins, connect with extiline*/
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
		GPIO_InitStructure.Pull = GPIO_NOPULL;

		GPIO_InitStructure.Pin = BTN1_PIN;
		HAL_GPIO_Init(BTN1_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = BTN2_PIN;
		HAL_GPIO_Init(BTN3_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = BTN3_PIN;
		HAL_GPIO_Init(BTN3_PORT, &GPIO_InitStructure);

		/*Configure priority levels */
		HAL_NVIC_SetPriority((IRQn_Type) EXTI9_5_IRQn, 2, 1);
		HAL_NVIC_EnableIRQ((IRQn_Type) EXTI9_5_IRQn);
	}

	static TIM_HandleTypeDef TIM_TimeBaseStructure;

private:
	static const uint32_t LONG_TIME_IN_MS = 1000;
	static const uint32_t MEDIUM_TIME_IN_MS = 1000 - 400;	//wait for 100ms
	state tempButtons[3] = { noPush, noPush, noPush };
	state oldButtons[3] = { noPush, noPush, noPush };
	static volatile state buttons[3];
	static volatile uint32_t buttonsTime[3];

	void InitTimer();
};

extern GPIO io_module;

#endif
