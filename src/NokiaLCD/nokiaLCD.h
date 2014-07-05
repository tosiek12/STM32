#ifndef NOKIALCD_H_
#define NOKIALCD_H_
#include "../Delay/delay.h"
#include "english_6x8_pixel.h"
#include "bmp_pixel.h"
//Folder: Nokia_5110_Display_Lib_for_STM32_GCC\LCD5110_Lib_Ifalt32-2.0

class NokiaLCD {
public:
	NokiaLCD() {
		Initialize();
	}

	void Clear(void);
	void ClearLine(uint8_t line);

	void GoTo(uint8_t X, uint8_t Y) {
		uint8_t x = 6 * X;	//Tego nie ma w innym
		LCD_write_byte(0x40 | Y, 0);	//Column
		LCD_write_byte(0x80 | x, 0);	//Row
	}

	void WriteText(char *s);
	void WriteTextInversed(char *s);
	void WriteTextXY(char *s, uint8_t X, uint8_t Y);

	void WriteNumberInDec(uint16_t b) {
		uint8_t datas[3];

		datas[0] = b / 1000;
		b = b - datas[0] * 1000;
		datas[1] = b / 100;
		b = b - datas[1] * 100;
		datas[2] = b / 10;
		b = b - datas[2] * 10;
		datas[3] = b;

		//Change number to its ASCII code.
		datas[0] += 48;
		datas[1] += 48;
		datas[2] += 48;
		datas[3] += 48;

		WriteData(datas[0]);
		WriteData(datas[1]);
		WriteData(datas[2]);
		WriteData(datas[3]);
	}

	void WriteBMP() {
		draw_bmp_pixel(0, 0, AVR_bmp, 24, 40);
	}

	void draw_bmp_pixel(uint8_t X, uint8_t Y, const uint8_t *map, uint8_t Pix_x,
			uint8_t Pix_y);

	void WriteDataInversed(uint8_t c);
	void WriteData(uint8_t c);

private:
	//1- VCC,GND,SCE,RST,DC,DN(MOSI),SCLK,LED-8//
	GPIO_TypeDef * LCD5110_CS_PORT = GPIOC;
	const uint16_t LCD5110_CS_PIN = GPIO_PIN_9;

	GPIO_TypeDef * LCD5110_RST_PORT = GPIOC;
	const uint16_t LCD5110_RST_PIN = GPIO_PIN_8;

	GPIO_TypeDef * LCD5110_DC_PORT = GPIOC;
	const uint16_t LCD5110_DC_PIN = GPIO_PIN_7;

	GPIO_TypeDef * LCD5110_MO_PORT = GPIOC;
	const uint16_t LCD5110_MO_PIN = GPIO_PIN_6;

	GPIO_TypeDef * LCD5110_SCK_PORT = GPIOA;
	const uint16_t LCD5110_SCK_PIN = GPIO_PIN_8;

#define GPIO_CLK_ENABLE(); __GPIOA_CLK_ENABLE(); __GPIOC_CLK_ENABLE();

	/*
	 * Before, sysTick must be initialized.
	 * It use delay_ms function!
	 */

	inline void __attribute__((always_inline)) Initialize(void) {
		GPIO_Config();
		Delay::delay_ms(10);

		RST(0); //LCD_RST = 0;
		Delay::delay_ms(1);
		RST(1); //LCD_RST = 1;

		CS(0); //SPI_CS = 0;
		Delay::delay_ms(1);
		CS(1); //SPI_CS = 1;

		Delay::delay_ms(1);

#define LCD5110_BIAS_1_100 0x10	//n=7 1:100
#define LCD5110_BIAS_1_80 0x11	//n=6 1:80
#define LCD5110_BIAS_1_65 0x12	//n=5 1:65/1:65
#define LCD5110_BIAS_1_48 0x13	//n=4 1:48
#define LCD5110_BIAS_1_40 0x14	//n=3 1:40/1:34
#define LCD5110_BIAS_1_24 0x15	//n=2 1:24
#define LCD5110_BIAS_1_18 0x16	//n=1 1:18/1:16
#define LCD5110_BIAS_1_10 0x17	//n=0 1:10/1:9/1:8

//Must be adjusted to the temperature of ambient.
#define LCD5110_TEMPERATURE_COEFFICIENT0 0x04
#define LCD5110_TEMPERATURE_COEFFICIENT1 0x05
#define LCD5110_TEMPERATURE_COEFFICIENT2 0x06
#define LCD5110_TEMPERATURE_COEFFICIENT3 0x07

#define LCD5110_CommandSet_Extended 0x21	//H = 1
#define LCD5110_CommandSet_Basic 0x20	//H = 0
#define LCD5110_NormalMode 0x0C	//H = 0

		LCD_write_byte(LCD5110_CommandSet_Extended, 0);
		LCD_write_byte(0xC0, 0); //Value of Vop(controls contrast) = (0x80 | 7-bit Vop value )
		LCD_write_byte(LCD5110_TEMPERATURE_COEFFICIENT2, 0);
		LCD_write_byte(LCD5110_BIAS_1_40, 0);
		LCD_write_byte(LCD5110_CommandSet_Basic, 0);
		Clear();				//Clear LCD
		LCD_write_byte(LCD5110_NormalMode, 0);//enable normal display (dark on light), horizontal addressing
		CS(0);	//SPI_CS = 0;
	}
	//Define the LCD Operation function
	void LCD_write_byte(uint8_t dat, uint8_t command);

	//Define the hardware operation function
	inline void __attribute__((always_inline)) GPIO_Config(void) {
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_CLK_ENABLE();

		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_NOPULL;

		GPIO_InitStructure.Pin = LCD5110_CS_PIN;
		HAL_GPIO_Init(LCD5110_CS_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = LCD5110_RST_PIN;
		HAL_GPIO_Init(LCD5110_RST_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = LCD5110_DC_PIN;
		HAL_GPIO_Init(LCD5110_DC_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = LCD5110_MO_PIN;
		HAL_GPIO_Init(LCD5110_MO_PORT, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = LCD5110_SCK_PIN;
		HAL_GPIO_Init(LCD5110_SCK_PORT, &GPIO_InitStructure);
	}
	inline void __attribute__((always_inline)) SCK(uint8_t temp) {
		if (temp) {
			HAL_GPIO_WritePin(LCD5110_SCK_PORT, LCD5110_SCK_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCD5110_SCK_PORT, LCD5110_SCK_PIN,
					GPIO_PIN_RESET);
		}
	}

	inline void __attribute__((always_inline)) MO(uint8_t temp) {
		if (temp) {
			HAL_GPIO_WritePin(LCD5110_MO_PORT, LCD5110_MO_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCD5110_MO_PORT, LCD5110_MO_PIN, GPIO_PIN_RESET);
		}
	}
	inline void __attribute__((always_inline)) CS(uint8_t temp) {
		if (temp) {
			HAL_GPIO_WritePin(LCD5110_CS_PORT, LCD5110_CS_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCD5110_CS_PORT, LCD5110_CS_PIN, GPIO_PIN_RESET);
		}
	}
	inline void __attribute__((always_inline)) RST(uint8_t temp) {
		if (temp) {
			HAL_GPIO_WritePin(LCD5110_RST_PORT, LCD5110_RST_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCD5110_RST_PORT, LCD5110_RST_PIN,
					GPIO_PIN_RESET);
		}
	}
	inline void __attribute__((always_inline)) DC(uint8_t temp) {
		if (temp) {
			HAL_GPIO_WritePin(LCD5110_DC_PORT, LCD5110_DC_PIN, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LCD5110_DC_PORT, LCD5110_DC_PIN, GPIO_PIN_RESET);
		}
	}

};

#endif

