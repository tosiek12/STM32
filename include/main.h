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
extern volatile uint8_t flagsComunicationInterface[10];
extern volatile uint8_t flagsHardware[10];

const uint8_t f_nonInit = 0;
const uint8_t f_configured = 1;
const uint8_t f_connected = 2;
const uint8_t f_connectedWithPC = 3;
const uint8_t f_connectedWithClient = 4;
const uint8_t f_deviceWorking = 5;

const uint8_t f_error = 10;
const uint8_t f_deviceNotFound = 11;

/* Global semaphores */
extern volatile uint8_t semaphore_timerInterrupt;


#endif /* MAIN_H_ */
