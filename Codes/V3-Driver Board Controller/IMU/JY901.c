/**
  ******************************************************************************
  * JY-901����
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "JY901.h"
#include "motor_pcb_interface.h"
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
struct SQuat		stcQuat;
float sAngle = 0;
short temp_Angle[5] = {0}, i = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
float Calculate_Angle()
{
	float sum = 0;
	short max = 0x8000, min = 0x7fff, j;
	temp_Angle[i++] = stcAngle.Angle[2];
	i %= 5;
	for (j = 0; j < 5; j++)
	{
		sum+=temp_Angle[j];
		if (max < temp_Angle[j])
			max = temp_Angle[j];
		if (min > temp_Angle[j])
			min = temp_Angle[j];
	}
	return ((sum - max - min)*60/32768);
}

void ParseSerialData(unsigned char ucData)
{
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucRxBuffer[12];
	
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
			case 0x52:	memcpy(&stcGyro, &ucRxBuffer[2], 8); break;			//���ٶ�								
			case 0x53:	memcpy(&stcAngle, &ucRxBuffer[2], 8);						//�Ƕ�
									sAngle = Calculate_Angle(); break;
			case 0x54:	memcpy(&stcMag, &ucRxBuffer[2], 8); break;			//�ų�
			case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[2], 8); break;	//�˿�״̬����
			case 0x56:	memcpy(&stcPress, &ucRxBuffer[2], 8); break;		//��ѹ
			case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;		//��γ��
			case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[2], 8); break;			//����
			case 0x59:	memcpy(&stcQuat, &ucRxBuffer[2], 8); break;			//��Ԫ��
			default:	break;
		}
		ucRxCnt=0;
	}
}

int16_t Inertia_Get_Angle_Yaw(void)
{
	return (int16_t)sAngle;
}