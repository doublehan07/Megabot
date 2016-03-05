/**
  ******************************************************************************
  * JY-901����
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "JY901.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
float 					sAngle = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void ParseSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	
	if (ucRxBuffer[0]!=0x55)  //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	
	if (ucRxCnt<11) {return;} //���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݹ�ͬ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x50:	memcpy(&stcTime, &ucRxBuffer[2], 8); break;			//ʱ��
			case 0x51:	memcpy(&stcAcc, &ucRxBuffer[2], 8); break;			//���ٶ�
			case 0x52:	memcpy(&stcGyro, &ucRxBuffer[2], 8);						//���ٶ�
									sAngle += stcGyro.w[2] / 163.84;
									break;
			case 0x53:	memcpy(&stcAngle, &ucRxBuffer[2], 8); break;		//�Ƕ�
			case 0x54:	memcpy(&stcMag, &ucRxBuffer[2], 8); break;			//�ų�
			case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[2], 8); break;	//�˿�״̬����
			case 0x56:	memcpy(&stcPress, &ucRxBuffer[2], 8); break;		//��ѹ
			case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;		//��γ��
			case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[2], 8); break;			//����
			default:	break;
		}
		ucRxCnt=0;
	}
}
