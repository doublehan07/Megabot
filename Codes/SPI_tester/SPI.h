#ifndef __SPI_H
#define __SPI_H

/* Includes ------------------------------------------------------------------*/
#include "15W204s.h"

typedef unsigned char uchar, uint8_t, bool;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;

#define TRUE 1 
#define FALSE 0
#define true 1 
#define false 0

//typedef unsigned long long uint64_t;
typedef struct{
	uint8_t high8;
	uint32_t low32;
}uint64_t;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Delay(unsigned int t);
void SPIInit(void);
uint8_t SPI_RW(uint8_t);
void select(void);                                                                          // selects the only slave for a transaction
void deselect(void);                                                                        // deselects the only slave after transaction

#endif
