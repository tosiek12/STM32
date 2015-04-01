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


GPIO IO_module;

// ----------------------------------------------------------------------------

//**end**//
