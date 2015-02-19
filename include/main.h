/*
 * main.h
 *
 *  Created on: 9 lis 2014
 *      Author: Antonio
 */

#ifndef MAIN_H_
#define MAIN_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

void Error_Handler(void);

extern volatile uint8_t trigger;
extern volatile uint8_t flags[10];

#endif /* MAIN_H_ */
