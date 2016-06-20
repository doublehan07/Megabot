/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RANGING_API_H
#define __RANGING_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
typedef uint64_t u64;

/* Exported macro ------------------------------------------------------------*/
/* Antenna delay values for 16 MHz PRF. */
//#define TX_ANT_DLY 16481 //Experiment value
//#define RX_ANT_DLY 16481 //Experiment value
#define TX_ANT_DLY 16438
#define RX_ANT_DLY 16438

#define RX_BUFFER_LENGTH 50

#define TIM_DIS_UNIT 4.69e-3

/* Exported constants --------------------------------------------------------*/
extern u32 frame_len;
extern u8 rx_buffer[RX_BUFFER_LENGTH];

/* Exported functions ------------------------------------------------------- */
void SPI_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void SPI_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void RESET_DW1000(void);
void Dwm1000_Init(void);

void Delay(__IO uint32_t nTime);
void SetTick(u32 tick);
u32 GetTick(void);

u8 SendMsg(u8 *buffer, u8 arrayLenth, u32 delayTime); //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)

#endif /* __RANGING_API_H */
