#include "wx_comm.h"

static struct Route route;

static float dx = 0;
static float dy = 0;
static u8 send_shape = 0;

static u8 Displacement_Msg_TX[8] = {0x7D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C};
static u8 Shape_Msg_TX[14] = {0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A};

struct Route GetRoute(void)
{
//	route.x = 0;
//	route.y = 0;
//	route.target_x = 1000;
//	route.target_y = 1000;
//	route.angle_0 = 1 * PI / 180;
	return route;
}

void SetShape(u16 *x, u16 *y)
{
	Shape_Msg_TX[1] = x[0] >> 8;
	Shape_Msg_TX[2] = (u8) x[0];
	Shape_Msg_TX[3] = y[0] >> 8;
	Shape_Msg_TX[4] = (u8) y[0];
	Shape_Msg_TX[5] = x[1] >> 8;
	Shape_Msg_TX[6] = (u8) x[1];
	Shape_Msg_TX[7] = y[1] >> 8;
	Shape_Msg_TX[8] = (u8) y[1];
	Shape_Msg_TX[9] = x[2] >> 8;
	Shape_Msg_TX[10] = (u8) x[2];
	Shape_Msg_TX[11] = y[2] >> 8;
	Shape_Msg_TX[12] = (u8) y[2];	
	send_shape = 1;
}

u8 ParseWXData(unsigned char wxData)
{
	static unsigned char ucRxCnt = 0;	
	static unsigned char ucRxBuffer[13];
	
	ucRxBuffer[ucRxCnt++] = wxData;
	
	if (ucRxBuffer[0] != 0x7F)
		ucRxCnt=0;
	
	if(ucRxCnt == 13)
	{
		if(ucRxBuffer[12] == 0x7E)
		{
			route.x = ucRxBuffer[1] << 8 | ucRxBuffer[2];
			route.y = ucRxBuffer[3] << 8 | ucRxBuffer[4];
			route.target_x = ucRxBuffer[5] << 8 | ucRxBuffer[6];
			route.target_y = ucRxBuffer[7] << 8 | ucRxBuffer[8];
			route.angle_0 = ucRxBuffer[9] << 8 | ucRxBuffer[10];
			route.angle_0 = route.angle_0 * PI / 180;
			if(ucRxBuffer[11])
				send_shape = 0;
			ucRxCnt = 0;
			return 1;
		}
		ucRxCnt = 0;
	}
	return 0;
}

void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3,USART_IT_RXNE) != RESET)
	{
		if(ParseWXData(USART_ReceiveData(USART3)))
		{
			u8 i = 0;
			u16 angle = getAngleDeg();
			Read_Displacement(&dx, &dy);
			Displacement_Msg_TX[1] = (short) dx >> 8;
			Displacement_Msg_TX[2] = (int8_t) dx;
			Displacement_Msg_TX[3] = (short) dy >> 8;
			Displacement_Msg_TX[4] = (int8_t) dy;
			Displacement_Msg_TX[5] = angle >> 8;
			Displacement_Msg_TX[6] = (u8) angle;
			for(; i<8; i++)
			{
				USART_SendData(USART3, Displacement_Msg_TX[i]);
				while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
			}
			if(send_shape)
			{
				for(i = 0; i < 14; i++)
				{
					USART_SendData(USART3, Shape_Msg_TX[i]);
					while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
				}
			}
		}
	}
}
