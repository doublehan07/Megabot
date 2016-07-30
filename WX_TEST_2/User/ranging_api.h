/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RANGING_API_H
#define __RANGING_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
typedef uint64_t u64;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void SPI_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void SPI_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void RESET_DW1000(void);
void Dwm1000_Init(void);

void Delay(__IO uint32_t nTime);
#endif /* __RANGING_API_H */
