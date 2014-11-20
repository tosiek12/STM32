#include "SpeedTester/speedTester.h"
#include "stm32f4xx_hal.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}
SpeedTester speedTester;

void SpeedTester::tic() {
	if( isUsed == 0) {
		isUsed = 1;
		TimHandle.Instance->CNT = 0x0000;	//Clear cnt
		__HAL_TIM_ENABLE(&TimHandle);
	} else {
		return; //SpeedTester is used in another place!
	}
}

uint32_t SpeedTester::toc() {
	__HAL_TIM_DISABLE(&TimHandle);
	isUsed = 0;
	return TimHandle.Instance->CNT;
}

uint32_t SpeedTester::delta() {
	res = TimHandle.Instance->CNT;
	TimHandle.Instance->CNT = 0;
	return res;
}

uint32_t SpeedTester::testTimeOfSending(uint32_t cnt) {
#define BUF_SIZE 100000
	char buf[BUF_SIZE];
	if(cnt < BUF_SIZE) {
		for( uint32_t i = 0; i < cnt; i++) {
			*(buf+i) = 48 + i%10;
		}
		buf[0] = 'S';
		buf[cnt-1] = 'E';

		speedTester.tic();
		VCP_write(buf, cnt);
		res = speedTester.toc();
	} else {
		res = 0;
	}
	return res;
}

