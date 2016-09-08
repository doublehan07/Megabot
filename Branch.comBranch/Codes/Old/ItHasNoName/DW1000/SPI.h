#ifndef __SPI_H
#define __SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SPIInit(void);
uint8_t SPI_RW(uint8_t);
void select(void);                                                                          // selects the only slave for a transaction
void deselect(void);                                                                        // deselects the only slave after transaction

#endif
