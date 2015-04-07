#include "GPIO/gpio.h"
#include "cortexm/ExceptionHandlers.h"

//Initializing static variables//
volatile GPIO::state GPIO::buttons[3] = { noPush, noPush, noPush };
volatile uint32_t GPIO::buttonsTime[3] = { 0 };
TIM_HandleTypeDef GPIO::TIM_TimeBaseStructure;

//**Buttons GPIO**//

/*
 * In main function is needed:
 * enum state tempButtons[3] = { noPush, noPush, noPush };
 * and its values must be refreshed at begging of the infinite loop
 * //strncpy((char*) tempButtons, (char*) buttons, 3 * sizeof(enum state)); //dont coppy whole table. ;(
 * tempButtons[0] = buttons[0];
 * tempButtons[1] = buttons[1];
 * tempButtons[2] = buttons[2];
 * To do proper action:
 * if (tempButtons[0] == shortPush) {
 //short push action
 } else if (tempButtons[0] == longPush) {
 //Long push action
 }
 */

/* LED5 - PD14,
 * LED6 - PD15
 * LED4 - PD12
 * LED3 - PD13
 */

void GPIO::SysTick_ButtonAction() {
	if (buttonsTime[0] > 0) {
		buttonsTime[0]--;
		if (buttonsTime[0] == MEDIUM_TIME_IN_MS) {
			buttons[0] = mediumPush;
		} else if (buttonsTime[0] == 0) {
			buttons[0] = longPush;
		}
	}
	if (buttonsTime[1] > 0) {
		buttonsTime[1]--;
		if (buttonsTime[1] == MEDIUM_TIME_IN_MS) {
			buttons[1] = mediumPush;
		} else if (buttonsTime[1] == 0) {
			buttons[1] = longPush;
		}
	}
	if (buttonsTime[2] > 0) {
		buttonsTime[2]--;
		if (buttonsTime[2] == MEDIUM_TIME_IN_MS) {
			buttons[2] = mediumPush;
		} else if (buttonsTime[2] == 0) {
			buttons[2] = longPush;
		}
	}
}

void GPIO::InitTimer() {
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

// ----- EXTI9_5_IRQHandler() ----------------------------------------------------

extern "C" void EXTI9_5_IRQHandler(void) {
	GPIO::EXTI_ButtonAction();
}

// ----------------------------------------------------------------------------

// ----- TIM_IRQHandler() ----------------------------------------------------

extern "C" void TIM4_IRQHandler(void) {
	if (__HAL_TIM_GET_ITSTATUS(&GPIO::TIM_TimeBaseStructure, TIM_IT_UPDATE ) != RESET) {
		GPIO::Timer_ButtonAction();

		__HAL_TIM_CLEAR_IT(&GPIO::TIM_TimeBaseStructure, TIM_IT_UPDATE);
	}

}


GPIO io_module;

// ----------------------------------------------------------------------------

//**end**//
