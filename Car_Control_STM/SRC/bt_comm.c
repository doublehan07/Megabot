#include "bt_comm.h"

static u16 shape_x[3] = {0, 0, 0};
static u16 shape_y[3] = {0, 0, 0};

static unsigned char ucRxCnt_Shape = 0;	
static unsigned char ucRxCnt_Cmd = 0;	

static u8 IsID0 = 0;
static u8 Cmd_Mode = 0; // 0--stop, 1--move forward, 2--move backward, 3--turn left, 4--turn right
static u8 Speed = 0;

u8 GetIsID0(void)
{
	return IsID0;
}

u8 GetCmd(void)
{
	return Cmd_Mode;
}

u8 GetSpeed(void)
{
	return Speed;
}

void ParseBTShape(unsigned char btData)
{
	static unsigned char ucRxBuffer[14];
	
	ucRxBuffer[ucRxCnt_Shape++] = btData;
	
	if (ucRxBuffer[0] != 0x7B)
		ucRxCnt_Shape=0;
	
	if(ucRxCnt_Shape == 14)
	{
		if(ucRxBuffer[13] == 0x7A)
		{
			shape_x[0] = ucRxBuffer[1] << 8 | ucRxBuffer[2];
			shape_y[0] = ucRxBuffer[3] << 8 | ucRxBuffer[4];
			shape_x[1] = ucRxBuffer[5] << 8 | ucRxBuffer[6];
			shape_y[1] = ucRxBuffer[7] << 8 | ucRxBuffer[8];
			shape_x[2] = ucRxBuffer[9] << 8 | ucRxBuffer[10];
			shape_y[2] = ucRxBuffer[11] << 8 | ucRxBuffer[12];
			SetShape(shape_x, shape_y);
			ucRxCnt_Cmd = 0;
		}
		ucRxCnt_Shape = 0;
	}
}

void ParseBTCmd(unsigned char btData)
{
	static unsigned char ucRxBuffer[4];
	
	ucRxBuffer[ucRxCnt_Cmd++] = btData;
	
	if (ucRxBuffer[0] != 0x7D)
		ucRxCnt_Cmd=0;
	
	if(ucRxCnt_Cmd == 4)
	{
		if(ucRxBuffer[3] == 0x7C)
		{
			Cmd_Mode = ucRxBuffer[1];
			Speed = ucRxBuffer[2];
			IsID0 = 1;
			ucRxCnt_Shape = 0;
		}
		ucRxCnt_Cmd = 0;
	}
}

void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE) != RESET)
	{
		u8 tmp = USART_ReceiveData(USART1);
		ParseBTShape(tmp);
		ParseBTCmd(tmp);
	}
}
