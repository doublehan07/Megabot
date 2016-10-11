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

/* Exported structs ----------------------------------------------------------*/
typedef struct
{
	u8 ID;
	u16 X;
	u16 Y;
	u16 Dist[3];
}Node;

/* Exported constants --------------------------------------------------------*/
extern u32 frame_len;
extern u8 rx_buffer[RX_BUFFER_LENGTH];

extern u64 tx_DeviceA_1;
extern u64 rx_DeviceA_2a;
extern u64 rx_DeviceA_2b;
extern u64 rx_DeviceA_2c;
extern u64 tx_DeviceA_3;

extern u64 rx_DeviceB_1;
extern u64 tx_DeviceB_2;
extern u64 rx_DeviceB_3;

extern Node ourNodes[4];

/* Exported functions ------------------------------------------------------- */
void SPI_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void SPI_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void RESET_DW1000(void);
void Dwm1000_Init(void);

void Delay(__IO uint32_t nTime);
void SetTick(u32 tick);
u32 GetTick(void);

u8 Brd_Msg(void);
u8 SendMsg(u8 *buffer, u8 arrayLenth, u32 delayTime); //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
u8 Safe_Receive(u32 timeout);
u8 Poll_Msg(u8 LBD, u8 UBD, u16 MyX, u16 MyY);
u8 Resp_Msg(u8 Target);
u8 Final_Msg(u16 delayTime, u64 TSa, u64 TSb1, u64 TSb2, u64 TSb3);

u64 get_tx_timestamp_u64(void);
u64 get_rx_timestamp_u64(void);
u64 get_sys_timestamp_u64(void);

void Range_Once(void); //调用测距函数，开始一轮定位

#endif /* __RANGING_API_H */
