/*
 * stm32f4_discovery_LIS3DSH.c
 *
 *  Created on: Dec 26, 2013
 *      Author: Pawel
 *
 *      IsNo 2013
 */

#include "stm32f4_discovery_LIS3DSH.h"

__IO uint32_t LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;

/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

static uint8_t _LIS3DSH_SendByte(uint8_t byte);
static void LIS3DSH_CalcAcceleration(LIS3DSH_OutXYZTypeDef* axes);

void LIS3DSH_SetOutputDataRate(uint8_t odr_selection) {
	uint8_t tmp;
	tmp = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG4_ADDR);
	tmp &= 0x0f;
	tmp |= odr_selection;
	LIS3DSH_WriteByte(LIS3DSH_CTRL_REG4_ADDR, tmp);
}
void LIS3DSH_SetFilterBandwidth(uint8_t filter_selection) {
	uint8_t tmp;
	tmp = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG5_ADDR);
	tmp &= (~_LIS3DSH_BW_MASK);
	tmp |= filter_selection;
	LIS3DSH_WriteByte(LIS3DSH_CTRL_REG5_ADDR, tmp);
}
void LIS3DSH_SetMeasuredRange(uint8_t range_selection) {
	uint8_t tmp;
	tmp = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG5_ADDR);
	tmp &= (~_LIS3DSH_FSCALE_MASK);
	tmp |= range_selection;
	LIS3DSH_WriteByte(LIS3DSH_CTRL_REG5_ADDR, tmp);
}
void LIS3DSH_SetOffsets(uint8_t xOffset, uint8_t yOffset, uint8_t zOffset) {
	LIS3DSH_WriteByte(LIS3DSH_OFFSET_X_ADDR, xOffset);
	LIS3DSH_WriteByte(LIS3DSH_OFFSET_Y_ADDR, yOffset);
	LIS3DSH_WriteByte(LIS3DSH_OFFSET_Z_ADDR, zOffset);
}
void LIS3DSH_AxesEnable(uint8_t axes) {
	uint8_t tmp;
	tmp = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG4_ADDR);
	tmp &= 0xf8;
	tmp |= axes;
	LIS3DSH_WriteByte(LIS3DSH_CTRL_REG4_ADDR, tmp);
}
void LIS3DSH_Int1Enable() {
	uint8_t tmp;
	tmp = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG3_ADDR);
	tmp |= (1 << _LIS3DSH_DR_EN) | (1 << _LIS3DSH_IEA)
			| (1 << _LIS3DSH_INT1_EN);
	LIS3DSH_WriteByte(LIS3DSH_CTRL_REG3_ADDR, tmp);
}

static void LIS3DSH_CalcAcceleration(LIS3DSH_OutXYZTypeDef* axes) {
	uint8_t ctrl = 0;
	ctrl = LIS3DSH_ReadByte(LIS3DSH_CTRL_REG5_ADDR);
	ctrl = (ctrl & _LIS3DSH_FSCALE_MASK);
	switch (ctrl) {
	case _LIS3DSH_FSCALE_2G:
		axes->x = (int16_t) (LIS3DSH_2G_FACTOR * (float)(axes->x));	//(in +/- mg)
		axes->y = (int16_t) (LIS3DSH_2G_FACTOR * (axes->y));	//(in +/- mg)
		axes->z = (int16_t) (LIS3DSH_2G_FACTOR * (axes->z));	//(in +/- mg)
		break;
	case _LIS3DSH_FSCALE_4G:
		axes->x = (int16_t) (LIS3DSH_4G_FACTOR * (axes->x));	//(in +/- mg)
		axes->y = (int16_t) (LIS3DSH_4G_FACTOR * (axes->y));	//(in +/- mg)
		axes->z = (int16_t) (LIS3DSH_4G_FACTOR * (axes->z));	//(in +/- mg)
		break;
	case _LIS3DSH_FSCALE_6G:
		axes->x = (int16_t) (LIS3DSH_6G_FACTOR * (axes->x));	//(in +/- mg)
		axes->y = (int16_t) (LIS3DSH_6G_FACTOR * (axes->y));	//(in +/- mg)
		axes->z = (int16_t) (LIS3DSH_6G_FACTOR * (axes->z));	//(in +/- mg)
		break;
	case _LIS3DSH_FSCALE_8G:
		axes->x = (int16_t) (LIS3DSH_8G_FACTOR * (axes->x));	//(in +/- mg)
		axes->y = (int16_t) (LIS3DSH_8G_FACTOR * (axes->y));	//(in +/- mg)
		axes->z = (int16_t) (LIS3DSH_8G_FACTOR * (axes->z));	//(in +/- mg)
		break;
	case _LIS3DSH_FSCALE_16G:
		axes->x = (int16_t) (LIS3DSH_16G_FACTOR * (axes->x));	//(in +/- mg)
		axes->y = (int16_t) (LIS3DSH_16G_FACTOR * (axes->y));	//(in +/- mg)
		axes->z = (int16_t) (LIS3DSH_16G_FACTOR * (axes->z));	//(in +/- mg)
		break;
	default:
		break;
	}
}
void LIS3DSH_ReadAxes(LIS3DSH_OutXYZTypeDef* axes) {
	uint8_t tmp = 0;
	/* Wait for new set of data */
	while (tmp == 0) {
		tmp = LIS3DSH_ReadByte(LIS3DSH_STATUS_ADDR);
		tmp &= (1 << _LIS3DSH_ZYXDA);	//ZYX data available
	}
	LIS3DSH_Read((uint8_t*) axes, LIS3DSH_OUT_X_L_ADDR, 6);
	axes->x += 3107;//programowa korekcja offsetu (wartosc byla za duza, by usunac sprzetowo)
	axes->y -= 3;//te juz sa dosc blisko, wiec nie ma duzej potrzeby by to robic
	axes->z -= 3;
	LIS3DSH_CalcAcceleration(axes);
}

uint8_t LIS3DSH_ReadByte(uint8_t addres) {
	uint8_t buffer;

	addres |= (uint8_t) READWRITE_CMD;
	LIS3DSH_CS_LOW();
	_LIS3DSH_SendByte(addres);
	buffer = _LIS3DSH_SendByte(DUMMY_BYTE);
	LIS3DSH_CS_HIGH();

	return buffer;
}
void LIS3DSH_Read(uint8_t* buf, uint8_t addres, uint16_t size) {
	while (size > 0x00) {
		*buf = LIS3DSH_ReadByte(addres);
		size--;
		buf++;
		addres++;
	}
}

void LIS3DSH_WriteByte(uint8_t addres, uint8_t value) {
	LIS3DSH_CS_LOW();
	_LIS3DSH_SendByte(addres);
	_LIS3DSH_SendByte(value);
	LIS3DSH_CS_HIGH();
}
void LIS3DSH_Write(uint8_t* buf, uint8_t addres, uint8_t size) {
	while (size > 0x00) {
		LIS3DSH_WriteByte(addres, *buf);
		size--;
		buf++;
		addres++;
	}
}

static SPI_HandleTypeDef SPI_HandleTypeDefStructure;

void LIS3DSH_Init() {
	__SPI1_CLK_ENABLE();

	/**SPI1 GPIO Configuration
	 PA5     ------> SPI1_SCK
	 PA6     ------> SPI1_MISO
	 PA7     ------> SPI1_MOSI
	 */

	GPIO_InitTypeDef GPIO_InitStructure;

// Enable the SPI
	LIS3DSH_SPI_CLK();
	LIS3DSH_SPI_SCK_GPIO_CLK();
	LIS3DSH_SPI_MISO_GPIO_CLK();
	LIS3DSH_SPI_MOSI_GPIO_CLK();
	LIS3DSH_SPI_CS_GPIO_CLK();

	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;

// SPI SCK pin configuration
	GPIO_InitStructure.Pin = LIS3DSH_SPI_SCK_PIN;
	HAL_GPIO_Init(LIS3DSH_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
// SPI  MOSI pin configuration
	GPIO_InitStructure.Pin = LIS3DSH_SPI_MOSI_PIN;
	HAL_GPIO_Init(LIS3DSH_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
// SPI MISO pin configuration
	GPIO_InitStructure.Pin = LIS3DSH_SPI_MISO_PIN;
	HAL_GPIO_Init(LIS3DSH_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
	// Configure GPIO PIN for Chip select */
	GPIO_InitStructure.Pin = LIS3DSH_SPI_CS_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(LIS3DSH_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
	//Select : Chip Select low*/
	LIS3DSH_CS_LOW();
// SPI configuration ----------------------------------------------------------------------------

	HAL_SPI_DeInit(&SPI_HandleTypeDefStructure);
	SPI_HandleTypeDefStructure.Instance = LIS3DSH_SPI;

	SPI_HandleTypeDefStructure.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_HandleTypeDefStructure.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_HandleTypeDefStructure.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_HandleTypeDefStructure.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_HandleTypeDefStructure.Init.NSS = SPI_NSS_SOFT;
	SPI_HandleTypeDefStructure.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	SPI_HandleTypeDefStructure.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_HandleTypeDefStructure.Init.TIMode = SPI_TIMODE_DISABLED;
	SPI_HandleTypeDefStructure.Init.CRCCalculation =
			SPI_CRCCALCULATION_DISABLED;
	SPI_HandleTypeDefStructure.Init.CRCPolynomial = 7;
	SPI_HandleTypeDefStructure.Init.Mode = SPI_MODE_MASTER;
	HAL_SPI_Init(&SPI_HandleTypeDefStructure);
	// Enable SPI1
	__HAL_SPI_ENABLE(&SPI_HandleTypeDefStructure);

	//Deselect : Chip Select high */
	LIS3DSH_CS_HIGH();

// Configure GPIO PINs to detect Interrupts */
	GPIO_InitStructure.Pin = LIS3DSH_SPI_INT1_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LIS3DSH_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = LIS3DSH_SPI_INT2_PIN;
	HAL_GPIO_Init(LIS3DSH_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}

static uint8_t _LIS3DSH_SendByte(uint8_t byte) {
	/* Loop while DR register in not emplty */
	LIS3DSHTimeout = LIS3DSH_FLAG_TIMEOUT;
	while (__HAL_SPI_GET_FLAG(&SPI_HandleTypeDefStructure,SPI_FLAG_TXE) == RESET) {
		if ((LIS3DSHTimeout--) == 0)
			return LIS3DSH_Timeout_UserCallback();
	}
	uint8_t buffer;
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&SPI_HandleTypeDefStructure, &byte, &buffer, 1, LIS3DSH_FLAG_TIMEOUT);
	if(status !=HAL_OK) {
		while(1) {
		}
	}
	/* Return the Byte read from the SPI bus */
	return buffer;
}
uint8_t LIS3DSH_Timeout_UserCallback(void) {
	/* Block communication and all processes */
	while (1) {
	}
}
