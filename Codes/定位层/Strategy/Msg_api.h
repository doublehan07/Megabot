/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSG_API_H
#define __MSG_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "deca_device_api.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	u16 Rect_Axis[2];
	u16 Polar_Axis[2];
	u8 ID;
	u8 MyStatus;
	/*
		0x00 - 待操作的未知节点
		0x01 - 主动测距点
		0x02 - 被动测距点
		0x03 - 被主动测距点指定的未知节点
		0x04 - 被被动测距点指定的未知节点
		0x05 - 已定位，状态翻转 //不响应广播信息
		0xFF - leader节点！
	*/
}MyInfo;

typedef struct
{
	u8 ID;
	u16 RectX;
	u16 RectY;
}NetInfo;

/* Exported macro ------------------------------------------------------------*/
#define netInfoSIZE	10

/* Antenna delay values for 16 MHz PRF. */
#define TX_ANT_DLY 16497 //Experiment value
#define RX_ANT_DLY 16497 //Experiment value
#define RX_BUF_LEN 30

/* Exported constants --------------------------------------------------------*/
extern double distance;
extern uint16_t distance_cm;
extern __IO uint8_t Usart_RX_flag;
extern __IO MyInfo myInfo;
extern NetInfo netInfo[netInfoSIZE];
extern __IO u8 netCnt;
extern dwt_config_t config;
extern uint8 rx_buffer[RX_BUF_LEN];

/* Exported functions ------------------------------------------------------- */
u8 Initiator_Ranging(uint8_t TargetID, u8 Times, u8 MyStatus, u8 which_freq);
u8 Receptor_Ranging(u8 *TargetID, u8 *Times, u8 *UpperStatus, u8 *which_freq);
u8 Corp_Ranging(uint8_t TargetID, u8 Times, u8 MyStatus, u8 which_freq, u16 Axis[4]);
u8 Receptor_Listening(void);
void Double_Buff_Recp_Listening(u8 *ID, u8 if_brd_use);

void ParseSerialData(unsigned char ucData);
void AI_Stategy(void);

void Broadcast_Msg(u8 CorpID, u8 *ID);
void CorpInfo_Msg(u8 CorpID, u16 *Axis);
void Selected_Msg(u8 SelectedID, u8 Frec);
void Boss_Msg(void);
void Resp_Msg(void);

void NetInfo_Init(u8 ID, u16 RectX, u16 RectY);
void MyInfo_Init(double Rect_Axis[2], double Polar_Axis[2], u8 MyStatus);
void Calculate_My_Pos(u16 tempa[2], u16 tempb[2], u16 *dist_Array, u16 *Axis);
void Decide_Our_Pos(u16 *myAxis, u16 *corpAxis, u16 dist, u8 CorpID);
void Leader_Strategy(void);
void Receptor_Strategy(void);
void Ranging_Strategy(void);
void FirstOne_Strategy(u8 CorpID);
void Coordianator_Strategy(u8 boss_ID);
void Change_Freq(u8 flag);

#endif /* __MSG_API */
