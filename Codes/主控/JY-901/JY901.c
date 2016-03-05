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
float 					sAngle = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void ParseSerialData(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
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
			case 0x52:	memcpy(&stcGyro, &ucRxBuffer[2], 8);						//角速度
									sAngle += stcGyro.w[2] / 163.84;
									break;
			case 0x53:	memcpy(&stcAngle, &ucRxBuffer[2], 8); break;		//角度
			case 0x54:	memcpy(&stcMag, &ucRxBuffer[2], 8); break;			//磁场
			case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[2], 8); break;	//端口状态数据
			case 0x56:	memcpy(&stcPress, &ucRxBuffer[2], 8); break;		//气压
			case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;		//经纬度
			case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[2], 8); break;			//地速
			default:	break;
		}
		ucRxCnt=0;
	}
}
