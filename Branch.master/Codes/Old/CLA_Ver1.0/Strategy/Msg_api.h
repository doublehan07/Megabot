/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MSG_API_H
#define __MSG_API_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "deca_device_api.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	double Rect_Axis[2];
	double Polar_Axis[2];
	u8 MyStatus;
	/*
		0x00 - ��������δ֪�ڵ�
		0x01 - ��������
		0x02 - ��������
		0x03 - ����������ָ����δ֪�ڵ�
		0x04 - ����������ָ����δ֪�ڵ�
		0x05 - �Ѷ�λ��״̬��ת //����Ӧ�㲥��Ϣ
	*/
	u8 RxTx_CodeNUm; //0 - 3, 1 - 4
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

/* Exported constants --------------------------------------------------------*/
extern double distance;
extern uint16_t distance_cm;
extern __IO uint8_t Usart_RX_flag;
extern __IO MyInfo myInfo;
extern NetInfo netInfo[netInfoSIZE];
extern __IO u8 netCnt;
extern dwt_config_t config;

/* Exported functions ------------------------------------------------------- */
uint8_t Initiator_Communication(uint8_t TargetID);
uint16_t Receptor_Communication(void);

void ParseSerialData(unsigned char ucData);
void Ranging_Stategy(void);

void Broadcast_Msg(u8 CorpID, u8 if_bdc_axis);
void Selected_Msg(u8 SelectedID, u8 Frec);
void Boss_Msg(void);
void Resp_Msg(void);

#endif /* __MSG_API */
