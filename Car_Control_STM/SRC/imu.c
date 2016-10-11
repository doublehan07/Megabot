#include "imu.h"

struct STime		 stcTime;
struct SAcc 	   stcAcc;
struct SGyro 		 stcGyro;
struct SAngle 	 stcAngle;
struct SMag 		 stcMag;
struct SDStatus  stcDStatus;
struct SPress 	 stcPress;
struct SLonLat 	 stcLonLat;
struct SGPSV 	 	 stcGPSV;
struct SQuat		 stcQuat;
float sAngle = 0;
float angle_offset = 0;
u8 angle_offset_set = 0;
u8 angle_pos = 0;
u8 angle_enough = 0;
short angles[N] = {0};

float NormAngleRad(float angle)
{
	while(angle < -PI)
		angle += 2*PI;
	while(angle > PI)
		angle += -2*PI;
	return angle;
}

void Calculate_Angle(void)
{
	float sum_sin = 0;
	float sum_cos = 0;
	float angle = 0;
	float angle_diff, angle_mean = 0, angle_max = 0, angle_min = 0;
	u8 j=0;
	angles[angle_pos++] = stcAngle.Angle[2];
	if(angle_pos == N)
		angle_enough = 1;
	angle_pos %= N;
	if(!angle_enough)
		return;
	for(j=0; j<N; j++)
	{
		angle = (float) angles[j] * PI / 32768;
		sum_sin += sin(angle);
		sum_cos += cos(angle);
	}
	angle_mean = atan2(sum_sin/N, sum_cos/N);
	for(j=0; j<N; j++)
	{
		angle = (float) angles[j] * PI / 32768;
		angle_diff = NormAngleRad(angle - angle_mean);
		if(angle_diff > angle_max)
		{
			angle_max = angle_diff;
		}
		if(angle_diff < angle_min)
		{
			angle_min = angle_diff;
		}
	}
	sum_sin -= sin(angle_min + angle_mean);
	sum_cos -= cos(angle_min + angle_mean);
	sum_sin -= sin(angle_max + angle_mean);
	sum_cos -= cos(angle_max + angle_mean);
	sAngle = NormAngleRad(atan2(sum_sin/(N-2), sum_cos/(N-2)) + ANGLE_OFFSET * angle_offset);
}

u8 SetAngleOffset(float angle)
{
	if(sAngle == 0)
		return angle_offset_set;
	if(angle_offset_set == 1)
		return angle_offset_set;
	angle_offset = NormAngleRad(angle - sAngle);
	angle_offset_set = 1;
	return angle_offset_set;
}

void ParseIMUData(unsigned char imuData)
{
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucRxBuffer[12];
	
	ucRxBuffer[ucRxCnt++] = imuData;
	
	if (ucRxBuffer[0] != 0x55)  //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	
	if (ucRxCnt < 11)
		return;
	switch(ucRxBuffer[1])
	{
		case 0x50:	memcpy(&stcTime, &ucRxBuffer[2], 8); break;				//时间
		case 0x51:	memcpy(&stcAcc, &ucRxBuffer[2], 8); break;				//加速度
		case 0x52:	memcpy(&stcGyro, &ucRxBuffer[2], 8); break;				//角速度								
		case 0x53:	memcpy(&stcAngle, &ucRxBuffer[2], 8); 
		            Calculate_Angle(); break;													//角度
		case 0x54:	memcpy(&stcMag, &ucRxBuffer[2], 8); break;				//磁场
		case 0x55:	memcpy(&stcDStatus, &ucRxBuffer[2], 8); break;		//端口状态数据
		case 0x56:	memcpy(&stcPress, &ucRxBuffer[2], 8); break;			//气压
		case 0x57:	memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;			//经纬度
		case 0x58:	memcpy(&stcGPSV, &ucRxBuffer[2], 8); break;				//地速
		case 0x59:	memcpy(&stcQuat, &ucRxBuffer[2], 8); break;				//四元数
		default:	break;
	}
	ucRxCnt = 0;
}

void USART2_IRQHandler(void)
{
	 if(USART_GetFlagStatus(USART2,USART_IT_RXNE) != RESET)
	 {
		 ParseIMUData(USART_ReceiveData(USART2));
	 }
}

float getAngleRad(void)
{
	return sAngle;
}

u16 getAngleDeg(void)
{
	if(!angle_enough)
		return 360;
	if(sAngle >= 0)
		return (u16)(sAngle * 180 / PI);
	else
		return (u16)((sAngle + 2*PI) * 180 / PI);
}
