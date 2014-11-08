/*
 * algorithms.h
 *
 *  Created on: 3 lis 2014
 *      Author: Antonio
 */

#ifndef ALGORITHMS_H_
#define ALGORITHMS_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

float32_t func(float32_t x);
float32_t integralTrapezoid(float32_t xp, float32_t xk, uint8_t numberOfIntervals);
float32_t integralSimpson(float32_t xp, float32_t xk, uint8_t numberOfIntervals);

#endif /* ALGORITHMS_H_ */
