/*
 * spi_sd.h
 *
 *  Created on: 2012-02-17
 *      Author: Grzesiek
 */

#ifndef SPI_SD_H_
#define SPI_SD_H_

#ifdef __cplusplus
extern "C" {
#endif

void SPI_SD_Init( void );
void disk_timerproc (void);
void testSDInit(void);

#ifdef __cplusplus
}
#endif


#endif /* SPI_SD_H_ */
