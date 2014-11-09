#include "Delay/delay.h"
#include "cortexm/ExceptionHandlers.h"
#include "stm32f4xx_hal.h"


volatile uint32_t Delay::delayCount;	//Create static value (default value is 0)


// ----- SysTick_Handler() ----------------------------------------------------

extern "C" void SysTick_Handler(void) {
	Delay::delay_SysTickAction();
	HAL_IncTick();

}

// ----------------------------------------------------------------------------
