#ifndef GPIO_H_
#define GPIO_H_

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

	/**
	 * Configure LED5, LED6 as output,
	 * LED3, LED4 as AF - for PWM from TIM4
	 */
	inline void __attribute__((always_inline)) InitLeds() {
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Periph clock enable */
		__GPIOD_CLK_ENABLE();	// The LEDs are on GPIOD

		//	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
		//	GPIO_InitStructure.Pin = LED5_PIN | LED6_PIN | LED4_PIN | LED3_PIN;
		//	GPIO_InitStructure.Mode = GPIO_Mode_OUT;
		//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		//	GPIO_InitStructure.Pull = GPIO_PuPd_NOPULL;
		//	GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* Configure PD14 (LED5) and PD15(LED6) in output pushpull mode */
		GPIO_InitStructure.Pin = GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

		// GPIOD, PIN_12 - LED4, PIN_13 - LED3, - for PWM.
		GPIO_InitStructure.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
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

	inline void __attribute__((always_inline)) InitTimer() {
		/* --------------------------------------------------------
		 TIM6 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1),
		 since APB1 prescaler is different from 1.
		 TIM6CLK = 2 * PCLK1
		 TIM6CLK = HCLK / 2 = SystemCoreClock /2
		 TIM6 Update event occurs each TIM6CLK/256
		 Note:
		 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
		 Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
		 function to update SystemCoreClock variable value. Otherwise, any configuration
		 based on this variable will be incorrect.
		 ----------------------------------------------------------- */

		const uint32_t CounterClk = 10000;	//100kHz
		const uint16_t OutputClk = 1000;	//1kHz
		const uint16_t Prescaler = (((SystemCoreClock / 2) / CounterClk) - 1);
		//ARR(TIM_Period) = (TIM3 counter clock / TIM3 output clock) - 1
		const uint32_t Period = ((CounterClk / OutputClk) - 1);

		/* TIM4 Periph clock enable */
		__TIM4_CLK_ENABLE();

		/* Time base configuration */
		TIM_TimeBaseStructure.Instance = TIM4;
		TIM_TimeBaseStructure.Init.Period = Period;
		TIM_TimeBaseStructure.Init.Prescaler = Prescaler;
		TIM_TimeBaseStructure.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TIM_TimeBaseStructure.Init.CounterMode = TIM_COUNTERMODE_UP;
		TIM_TimeBaseStructure.Init.RepetitionCounter = 0;
		HAL_TIM_Base_Init(&TIM_TimeBaseStructure);

		HAL_NVIC_SetPriority((IRQn_Type) TIM4_IRQn, 1, 1);
		HAL_NVIC_EnableIRQ((IRQn_Type) TIM4_IRQn);

		HAL_TIM_Base_Start_IT(&TIM_TimeBaseStructure);
	}
};

#endif
