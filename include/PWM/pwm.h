#ifndef PWM_H_
#define PWM_H_

#include "cmsis_device.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "main.h"

class PWM {
private:
public:
	TIM_HandleTypeDef TimHandle;
	uint32_t uwPulse1;
	uint32_t uwPulse2;
	uint32_t uwPulse3;
	uint32_t uwPulse4;
	PWM() {
		uwPulse1 = 0;
		uwPulse2 = 0;
		uwPulse3 = 0;
		uwPulse4 = 0;
	}

	void PWM_MspInit() {
		GPIO_InitTypeDef GPIO_InitStruct;

		/* GPIO Ports Clock Enable */
		__GPIOE_CLK_ENABLE();

		/**TIM1 GPIO Configuration
		 PE9     ------> TIM1_CH1
		 PE11    ------> TIM1_CH2
		 PE13    ------> TIM1_CH3
		 PE14    ------> TIM1_CH4
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	}

	void PWMInit() {
		TIM_MasterConfigTypeDef sMasterConfig;
		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
		TIM_OC_InitTypeDef sConfig;

		PWM_MspInit();

		/* TIMx Peripheral clock enable */
		__TIM1_CLK_ENABLE();

		const uint32_t CounterClk = 10000;	//Hz
		//TIM1CLK = 2 * APB2 clock (PCLK2)
		//PCLK2 = HCLK / 2
		//Prescaler = ((SystemCoreClock) / TIM1 counter clock) - 1
		const uint16_t Prescaler = (((SystemCoreClock) / CounterClk) - 1);
		//ARR(TIM_Period) = (TIM3 counter clock / TIM3 output clock) - 1
		const uint16_t OutputClk = 54;	//Hz
		const uint32_t Period = ((CounterClk / OutputClk) - 1);

		/* Set TIMx instance */
		TimHandle.Instance = TIM1;
		TimHandle.Init.Period = Period;
		TimHandle.Init.Prescaler = Prescaler;
		TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
		TimHandle.Init.RepetitionCounter = 0;
		if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
			Error_Handler();
		}

		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		HAL_TIM_ConfigClockSource(&TimHandle, &sClockSourceConfig);

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig);

		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakDeadTimeConfig);

		sConfig.OCMode = TIM_OCMODE_PWM1;
		sConfig.Pulse = 500;
		sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfig.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK) {
			Error_Handler();
		}

		sConfig.Pulse = 0;
		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK) {
			Error_Handler();
		}
	}

	void startPwmChannel(uint32_t channel) {
		if (HAL_TIM_PWM_Start(&TimHandle, channel) != HAL_OK) {
				Error_Handler();
			}
	}
	void startAllPwmChannels() {
		if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK) {
			Error_Handler();
		}
		if (HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK) {
			Error_Handler();
		}
	}

	void setChannelRawValue(uint8_t chanelNumber, uint32_t pulseValue) {

		if (chanelNumber == TIM_CHANNEL_1) {
			/* Set the Capture Compare Register value */
			TimHandle.Instance->CCR1 = pulseValue;
		}
	}

};

extern PWM pwm;

#endif

