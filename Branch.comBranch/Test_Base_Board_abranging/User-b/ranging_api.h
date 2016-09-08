/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RANGING_API_H
#define __RANGING_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Antenna delay values for 16 MHz PRF. */
//#define TX_ANT_DLY 16481 //Experiment value
//#define RX_ANT_DLY 16481 //Experiment value
#define TX_ANT_DLY 16415
#define RX_ANT_DLY 16415

/* Exported functions ------------------------------------------------------- */
void SPI_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void SPI_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void RESET_DW1000(void);
void Dwm1000_Init(void);

void Initiator_Communication(uint8_t TargetID);
void Receptor_Communication(void);

#endif /* __RANGING_API_H */
