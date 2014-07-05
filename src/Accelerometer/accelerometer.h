#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
#include "../NokiaLCD/nokiaLCD.h"

/* Exported macro ------------------------------------------------------------*/
#define ABS(x)                           ((x < 0) ? (-x) : x)
#define MAX(a,b)                         ((a < b) ? (b) : a)

extern volatile int16_t xZero, yZero, zZero;
extern volatile int16_t xActual, yActual, zActual;
extern volatile uint8_t acc_initialized;

void SysTick_UpdateAccelerometer();
void InitAccelerometer();
void Main_AccelerometerAction(NokiaLCD *pNokiaLCD);


#endif



