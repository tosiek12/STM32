#ifndef GPS_H_
#define GPS_H_

#include "cmsis_device.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()

/**USART1 GPIO Configuration
PA9     ------> USART1_TX
PA10     ------> USART1_RX
*/
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1

#define USARTx_IRQn ((IRQn_Type)USART1_IRQn)
/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

void GPS_Init();

struct gpsData_t {
	// time
	uint8_t hhmmss[6];
	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	// date
	uint8_t day;
	uint8_t month;
	uint8_t year;

	// position:
	uint8_t valid;
	int32_t lat;
	int32_t lon;
	uint16_t alt;

	//Orientation:
	uint16_t headingTruePRMC;
	uint16_t headingMagneticPVTG;
	uint16_t headingTruePVTG;
	uint16_t speedInKm_h;

	//Conection stats:
	uint8_t position3D;
	uint8_t modeGPSA;
	uint8_t sats;
	uint8_t satID[12];
	uint32_t dop;
	uint32_t hdop;
	uint32_t vdop;
};

extern struct gpsData_t gpsdata;

void GPS_Parse(struct gpsData_t *gpsdata, uint8_t *buf, uint8_t len);

#endif
