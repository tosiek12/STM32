#ifndef DELAY_H_
#define DELAY_H_

#include "cmsis_device.h"

class Delay {
private:
	static constexpr uint32_t FREQUENCY_HZ = 1000000u;	//1Mhz
	static volatile uint32_t delayCount;

public:
	static void initialize() {
		// Use SysTick as reference for the delay loops.
		//168/8 = 21MHz
		//21MHz/21=1MHz => T=1ms;
		if (SysTick_Config(SystemCoreClock / FREQUENCY_HZ)) {
			//Capture error
			while (1) {
			}
		}
	}

	inline static void delay_SysTickAction(void) {
		// Decrement to zero the counter used by the delay routine.
		if (delayCount != 0u) {
			--delayCount;
		}
	}

	void static delay_ms(uint32_t timeInMs) {
		delayCount = timeInMs * 1000;
		while (delayCount) {
		}
	}
	void static delay_us(uint32_t timeInUs) {
		delayCount = timeInUs;
		while (delayCount) {
		}
	}
};

#endif

