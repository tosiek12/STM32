#include "Delay/delay.h"
#include "cortexm/ExceptionHandlers.h"
#include "stm32f4xx_hal.h"

#include "SD/spi_sd.h"

volatile uint32_t Delay::delayCount;	//Create static value (default value is 0)

// ----- SysTick_Handler() ----------------------------------------------------

extern "C" void SysTick_Handler(void) {
	static uint8_t cnter = 0;
	if((++cnter) == 10) {
		disk_timerproc();
		cnter = 1;
	}
	Delay::delay_SysTickAction();
	HAL_IncTick();


}

// ----------------------------------------------------------------------------
