#ifndef SPEEDTESTER_H_
#define SPEEDTESTER_H_

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

class SpeedTester {
private:
	uint32_t res;
	uint8_t isUsed;
public:
	TIM_HandleTypeDef TimHandle;
	SpeedTester() {
		isUsed = 0;
		res = 0;
		/* TIMx Peripheral clock enable */
		__TIM5_CLK_ENABLE();

		const uint32_t CounterClk = 1000000;	//Hz

		//TIM3CLK = 2 * APB1 clock (PCLK1)
		//PCLK1 = HCLK / 4

		//Prescaler = ((SystemCoreClock/2) / TIM3 counter clock) - 1
		const uint16_t Prescaler = (((SystemCoreClock / 2) / CounterClk) - 1);

		//ARR(TIM_Period) = (TIM3 counter clock / TIM3 output clock) - 1
		//const uint16_t OutputClk = 10000;	//Hz
		//const uint32_t Period = ((CounterClk / OutputClk) - 1);

		/* Set TIMx instance */
		TimHandle.Instance = TIM5;
		TimHandle.Init.Period = UINT32_MAX - 1;
		TimHandle.Init.Prescaler = Prescaler;
		TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
		TimHandle.Init.RepetitionCounter = 0;
		if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
			/* Initialization Error */
			while (1) {
			};
		}
		__HAL_TIM_DISABLE(&TimHandle); //Stop timer

		TIM_MasterConfigTypeDef sMasterConfig;
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);
	}
	void tic();
	uint32_t toc();

	uint8_t getIsUsed() {
		return isUsed;
	}
	uint32_t delta();
	uint32_t testTimeOfSending(uint32_t );

};
extern SpeedTester speedTester;
#endif

