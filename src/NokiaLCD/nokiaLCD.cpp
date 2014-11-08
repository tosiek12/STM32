#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "nokiaLCD.h"
#include "../Delay/delay.h"

void NokiaLCD::WriteData(uint8_t c) {
	uint8_t line;
	c -= 32;
	for (line = 0; line < 6; line++) {
		LCD_write_byte(font6x8[c][line], 1);
	}
}

void NokiaLCD::WriteDataInversed(uint8_t c) {
	uint8_t line;
	c -= 32u;
	for (line = 0u; line < 6u; line++) {
		LCD_write_byte(~font6x8[c][line], 1u);
	}
}

/*-----------------------------------------------------------------------
 LCD_draw_map      : Bitmap drawing function
 input parameter
 starting point X,Y
 *map    bitmap data
 Pix_x   height
 Pix_y   width
 -----------------------------------------------------------------------*/
void NokiaLCD::draw_bmp_pixel(uint8_t X, uint8_t Y,const uint8_t  *map, uint8_t Pix_x,
		uint8_t Pix_y) {
	uint8_t i, n;
	uint8_t row;

	if (Pix_y % 8 == 0)
		row = Pix_y / 8;      //calculate how many line is needed
	else
		row = Pix_y / 8 + 1;

	for (n = 0; n < row; n++) {
		GoTo(X, Y);
		for (i = 0; i < Pix_x; i++) {
			LCD_write_byte(map[i + n * Pix_x], 1);
		}
		++Y;	//change line
	}
}

void NokiaLCD::WriteText(char *s) {
	while (*s != '\0') {
		WriteData(*s);
		++s;
	}
}

void NokiaLCD::WriteTextXY(char *s, uint8_t X, uint8_t Y) {
	GoTo(X,Y);
	while (*s != '\0') {
		WriteData(*s);
		++s;
	}
}

void NokiaLCD::WriteTextInversed(char *s) {
	while (*s != '\0') {
		WriteDataInversed(*s);
		++s;
	}
}

void NokiaLCD::Clear() {
//	LCD_write_byte(0x0c, 0);
//	LCD_write_byte(0x80, 0); //Inna wersja - tego nie bylo wczesniej
	for (uint16_t i = 0; i < 504; i++) {	//6x84 = 504 pixeli
		LCD_write_byte(0, 1);
	}
}

void NokiaLCD::ClearLine(uint8_t line) {
//	LCD_write_byte(0x0c, 0);
//	LCD_write_byte(0x80, 0); //Inna wersja - tego nie bylo wczesniej

	GoTo(0,line);
	for (uint16_t i = 0; i < 84; i++) {	//6x84 = 504 pixeli
		LCD_write_byte(0, 1);
	}
}

void NokiaLCD::LCD_write_byte(uint8_t dat, uint8_t command) {
	uint8_t i;
	uint8_t cnt;
	CS(0);	//SPI_CS = 0;
//	Delay::delay_us(1);//10
	cnt = 100; while(--cnt) { asm volatile("nop"); }
	if (command == 0)
		DC(0);	//LCD_DC = 0;
	else
		DC(1);	//LCD_DC = 1; - write data to RAM
//	Delay::delay_us(1);//10
	cnt = 100; while(--cnt) { asm volatile("nop"); }
	for (i = 0; i < 8; i++) {
		MO(dat & 0x80);	//SPI_MO = dat & 0x80;
		dat = dat << 1;
		SCK(0);	//SPI_SCK = 0;
//		Delay::delay_us(1);
		cnt = 100; while(--cnt) { asm volatile("nop");}
		SCK(1);	//SPI_SCK = 1;
//		Delay::delay_us(1);
		cnt = 100; while(--cnt) { asm volatile("nop");}
	}
	CS(1);	//SPI_CS = 1;
}

