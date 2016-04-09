/**
  ******************************************************************************
  * JY-901代码
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
struct SQuat		stcQuat;
u16 sAngle = 0;
short temp_Angle[5] = {0}, i = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
u16 Calculate_Angle(void) //角度滤波，避免抖动
{
	static u8 counter_once = 0;
	float result = 0.0;
	short max = 0x8000, min = 0x7fff, j;
	int32_t sum = 0;
	u16 return_val;
	
	if(counter_once == 0)
	{
		for(i = 0; i < 5; i++)
			temp_Angle[i] = stcAngle.Angle[2];
		i = 1;
		counter_once++;
	}
	else
	{
		temp_Angle[i++] = stcAngle.Angle[2];
		i %= 5;
	}
	for (j = 0; j < 5; j++)
	{
		sum += temp_Angle[j];
		if (max < temp_Angle[j])
			max = temp_Angle[j];
		if (min > temp_Angle[j])
			min = temp_Angle[j];
	}
	result  = (sum - max - min) * 60.0 / 32768.0; //按照datasheet给的计算公式
	return_val = ((u16)result + 360) % 360; //角度输出为0-360整型就可以啦
	return return_val;
}

void ParseSerialData(unsigned char ucData)
{
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucRxBuffer[12];
	
	ucRxBuffer[ucRxCnt++]=ucData;
	
	if (ucRxBuffer[0]!=0x55)  //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	
	if (ucRxCnt<11) {return;} //数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
			case 0x50:	memcpy(&stcTime, &ucRxBuffer[2], 8); break;			//时间
			case 0x51:	memcpy(&stcAcc, &ucRxBuffer[2], 8); break;			//加速度
			case 0x52:	memcpy(&stcGyro, &ucRxBuffer[2], 8); break;			//角速度								
			case 0x53:	memcpy(&stcAngle, &ucRxBuffer[2], 8);						//角度
									sAngle = Calculate_Angle(); break;
			case 0x54:	memcpy(&stcMag, &ucRxBuffer[2], 8); break;			//磁场
			case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[2], 8); break;	//端口状态数据
			case 0x56:	memcpy(&stcPress, &ucRxBuffer[2], 8); break;		//气压
			case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;		//经纬度
			case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[2], 8); break;			//地速
			case 0x59:	memcpy(&stcQuat, &ucRxBuffer[2], 8); break;			//四元数
			default:	break;
		}
		ucRxCnt=0;
	}
}
