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
#define TX_ANT_DLY 16438
#define RX_ANT_DLY 16438

#define BUFFER_LENGTH 30 //The length of rx_buffer

/* Exported functions ------------------------------------------------------- */
void SPI_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void SPI_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void RESET_DW1000(void);
void Dwm1000_Init(void);

//rx_buffer[30]
/*CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
	    		| CoodinateY | Dist1(RESV) | Dist2(RESV) | CommCount | (CRC) | (CRC)
	0x0A | 0x01 | 0x02 | 0x04 | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 | (0x0D) | (0x0A)
*/
u8 Initiator_Ranging(u8 *data, u8 length); //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
u16 Receptor_Ranging(u8 delay, u8 leftnumber); //return IF_FAIL_GET_DIST(distance_cm, 0xFFFF - F)
u8 Receptor_Listening(u8 *rx_buffer, u8 *length, u8 *emergency_stop, u16 timeout); //return If_FAIL_COMMUNICATION(0x00 - T, 0xFF - F)
u8 SendMsg(u8 *pointer, u8 arrayLenth); //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
void Delay(__IO uint32_t nTime);

#endif /* __RANGING_API_H */
