#include "PWM/pwm.h"
#include "cortexm/ExceptionHandlers.h"
#include "stm32f4xx_hal.h"

PWM pwm;

extern "C" void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_NVIC_ClearPendingIRQ((IRQn_Type) TIM1_CC_IRQn);
  HAL_TIM_IRQHandler(&pwm.TimHandle);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}
