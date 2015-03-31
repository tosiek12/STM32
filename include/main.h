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
/* Global variables */
/* Flags vectors */
extern volatile uint8_t flagsComunicationInterface[10];
extern volatile uint8_t flagsHardware[10];
/* Posible flags states */
const uint8_t f_nonInit = 0;
const uint8_t f_configured = 1;
const uint8_t f_connected = 2;
const uint8_t f_connectedWithPC = 3;
const uint8_t f_connectedWithClient = 4;
const uint8_t f_deviceWorking = 5;

const uint8_t f_error = 10;
const uint8_t f_deviceNotFound = 11;
/* indexes for devices */
const uint8_t f_device_accelerometer = 0;
const uint8_t f_device_gyroscope = 1;
const uint8_t f_device_magnetometer = 2;
const uint8_t f_device_pressure = 3;
const uint8_t f_device_gps = 4;
const uint8_t f_device_sdCard = 5;

const uint8_t f_interface_sensors = 0;	//SPI1
const uint8_t f_interface_gps = 1;	//UART1
const uint8_t f_interface_sdCard= 2;	//SPI2
const uint8_t f_interface_USB = 3;	//USB

/* Semaphores */
extern volatile uint8_t semaphore_timerInterrupt;



#endif /* MAIN_H_ */
