/**
  ******************************************************************************
  * ��λ��ͨ��ָ��
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "communication.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char DW_RX_Buffer[6] = {0, 0, 0, 0, 0, 0};
Parse_DW_Data parseData;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Parse_DW1000_Data(unsigned char ucData)
{
	//����λ��ͨ�Ÿ�ʽ��0x0A | 0xCF | TargetID | 2-bit dist | 0xFC
	
	static u8 ucRxCnt = 0;	
	
	DW_RX_Buffer[ucRxCnt++]=ucData;
	
	if (DW_RX_Buffer[0] != 0x0A)  //��ʼ����У��I�������������¿�ʼѰ��0x0A����ͷ
	{
		ucRxCnt = 0;
		return;
	}
	else if (ucRxCnt == 2 && DW_RX_Buffer[1] != 0xCF) //��ʼ����У��II - 0xCF
	{
		ucRxCnt = 0;
		return;
	}
	
	if (ucRxCnt < 6) {return;} //���ݲ���6�����򷵻�
	else
	{
		memcpy(&parseData, &DW_RX_Buffer[2], 3);
		ucRxCnt=0;
	}
}
