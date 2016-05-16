/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSG_API_H
#define __MSG_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "deca_device_api.h"
#include "port.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	u16 Rect_Axis[2];
	u16 Polar_Axis[2];
	u8 ID;
	u8 MyStatus;
	/*
		0x00 - ��������δ֪�ڵ�
		0x01 - ��������
		0x02 - ��������
		0x03 - ����������ָ����δ֪�ڵ�
		0x04 - ����������ָ����δ֪�ڵ�
		0x05 - �Ѷ�λ��״̬��ת //����Ӧ�㲥��Ϣ
		0xFF - leader�ڵ㣡
	*/
}MyInfo;

typedef struct
{
	u8 ID;
	int16 RectX;
	int16 RectY;
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
u8 Corp_Ranging(uint8_t TargetID, u8 Times, u8 MyStatus, u8 which_freq, int16 Axis[4]);
u8 Receptor_Listening(u16 timeout);
void Double_Buff_Recp_Listening(u8 *ID, u8 if_brd_use);

void ParseSerialData(unsigned char ucData);
void AI_Stategy(void);

void Broadcast_Msg(u8 CorpID, u8 *ID);
void CorpInfo_Msg(u8 CorpID, int16 *Axis);
void Selected_Msg(u8 SelectedID, u8 Frec);
void Boss_Msg(void);
void Resp_Msg(void);
void Last_One_Msg(u8 LastID);
void Stop_Waiting_Msg(void);

void NetInfo_Init(u8 ID, int16 RectX, int16 RectY);
void MyInfo_Init(double Rect_Axis[2], double Polar_Axis[2], u8 MyStatus);
void Calculate_My_Pos(int16 tempa[2], int16 tempb[2], u16 *dist_Array, int16 *Axis);
void Cal_Mypos_Triangle(int16 posA[2], int16 posB[2], int16 posC[2], u16 dist_Array[3], int16 Axis[2]);
void Decide_Our_Pos(int16 *myAxis, int16 *corpAxis, u16 dist, u8 CorpID);
void Leader_Strategy(void);
void Receptor_Strategy(void);
void Ranging_Strategy(void);
void FirstOne_Strategy(u8 CorpID);
void Coordianator_Strategy(u8 boss_ID);
void LastOne_Strategy(void);
void Change_Freq(u8 flag);

#endif /* __MSG_API */