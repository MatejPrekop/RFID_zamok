/*
 * spi.h
 *
 *  Created on: 17. 12. 2016
 *      Author: Mallto
 */

#ifndef SPI_H_
#define SPI_H_


void initSPI(void);
unsigned char readWriteSPI(unsigned char txData);

//Example of CS use
void initCS_Pin(void);
void device_Select(void);
void device_Unselect(void);

void initCD_Pin(void);
void cd_set(void);
void cd_reset(void);

void initRES_Pin(void);
void res_set(void);
void res_reset(void);

#endif /* SPI_H_ */
