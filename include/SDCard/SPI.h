/*
 * I2C.h
 *
 *  Created on: 11 lip 2014
 *      Author: Antonio
 */

#ifndef I2C_H_
#define I2C_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "../Delay/delay.h"

#define TIMEOUT 1000

class SPI {
private:
	static SPI_HandleTypeDef hspi1;
	static uint8_t initialized;

	static void HAL_SPI_MspInit() {
		GPIO_InitTypeDef GPIO_InitStruct;

		/* Peripheral clock enable */
		__SPI2_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();

		/**SPI2 GPIO Configuration
		PC2     ------> SPI2_MISO
		PC3     ------> SPI2_MOSI
		PB10     ------> SPI2_SCK
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;	//by³o nopullup
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	}

	static void HAL_SPI_MspDeInit() {

		/* Reset peripherals */
		__SPI1_FORCE_RESET();
		Delay::delay_ms(1);
		__SPI1_RELEASE_RESET();

		/* Peripheral clock disable */
		__SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	}
public:
	SPI() { };

	static void SPI_Init() {
		HAL_SPI_MspInit();

		__SPI1_CLK_ENABLE();

//		/* Reset peripherals */
		__SPI1_FORCE_RESET();
		Delay::delay_ms(1);
		__SPI1_RELEASE_RESET();

		__SPI1_CLK_ENABLE();

		hspi1.Instance = SPI2;
		hspi1.Init.Mode = SPI_MODE_MASTER;
		hspi1.Init.Direction = SPI_DIRECTION_2LINES;
		hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
		hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
		hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
		hspi1.Init.NSS = SPI_NSS_SOFT;
		hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
		hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
		hspi1.Init.CRCPolynomial = 7;

		if(HAL_SPI_Init(&hspi1) != HAL_OK) {
			/* Initialization Error */
			while(true) {

			};
		}

		Delay::delay_ms(2);
		initialized = 1;
	}

	static uint8_t TM_SPI_Send(uint8_t data) {
		uint8_t temp = 0;
		HAL_StatusTypeDef state = HAL_ERROR;
		state = HAL_SPI_TransmitReceive(&hspi1, &data, &temp, 1, TIMEOUT);
		//Fill output buffer with data
		if(state != HAL_OK) {
			while(true) { };
		}
		return temp;

//	    hspi1.Instance->DR = data;
//		//Wait for transmission to complete
//		while (SPI_WaitOnFlagUntilTimeout(hspi1.Instance, SPI_I2S_FLAG_TXE, RESET, 10) != HAL_OK);
//		//Wait for received data to complete
//		while (!SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE));
//		//Wait for SPI to be ready
//		while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY));
//		//Return data from buffer
//		return hspi1.Instance->DR;
	}

	static void TM_SPI_SendMulti(uint8_t* dataOut, uint8_t* dataIn, uint16_t count) {
		HAL_StatusTypeDef state = HAL_ERROR;
		state = HAL_SPI_TransmitReceive(&hspi1, dataOut, dataIn, count, TIMEOUT);
		if(state != HAL_OK) {
			while(true) { };
		}
	}

	static void TM_SPI_WriteMulti(uint8_t* dataOut, uint16_t count) {
		HAL_StatusTypeDef state = HAL_ERROR;
		state = HAL_SPI_Transmit(&hspi1, dataOut, count, TIMEOUT);
		if(state != HAL_OK) {
			while(true) { };
		}
	}

	static void TM_SPI_ReadMulti(uint8_t* dataIn, uint8_t dummy, uint16_t count) {
		HAL_StatusTypeDef state = HAL_ERROR;
		volatile uint8_t temp = 1;
		state = HAL_SPI_Receive(&hspi1, dataIn, count, TIMEOUT);
		// UWAGA ! WCHODZI DO TIMEOUT!!
		if(state != HAL_OK) {
			while(true) {
				temp = state;
			};
		}
	}

};

#endif /* I2C_H_ */
