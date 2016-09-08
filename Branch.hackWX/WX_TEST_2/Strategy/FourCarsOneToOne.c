/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FourCarsOneToOne.h"
#include "initiator.h"
#include "receptor.h"
#include "ranging_api.h"

#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 d01;
static u16 d02;
static u16 d03;
static u16 d12;
static u16 d13;
static u16 d23;

static u8 n3Msg[] = {0, 0, 0, 0, 0, 0};
static u8 n2Msg[] = {0, 0, 0, 0};
static u8 n1Msg[] = {0, 0};

static u8 n1Axis[] = {0x0D, 0, 0, 0, 0, 0x0A}; //xH | xL | yH | yL
static u8 n2Axis[] = {0x0D, 0, 0, 0, 0, 0x0A}; //xH | xL | yH | yL
static u8 n3Axis[] = {0x0D, 0, 0, 0, 0, 0x0A}; //xH | xL | yH | yL
static u8 n4Axis[] = {0x0D, 0, 0, 0, 0, 0x0A}; //xH | xL | yH | yL

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void FourCarsLocalization(void)
{
	static double temp_calc, comp1, comp2;
	static int16_t temp_trans;
	static int16_t x1, y1, x2, y2;
	static u8 i;
	
	//The first turn
	if(MyID == 0x00)
	{
		while(Initiator_Ranging(0x03) == 0xFF);
		while(Initiator_Ranging(0x02) == 0xFF);
		while(Initiator_Ranging(0x01) == 0xFF);
	}
	else if(MyID == 0x01)
	{
		while(Receptor_Ranging(&d01) == 0xFF);
	}
	else if(MyID == 0x02)
	{
		while(Receptor_Ranging(&d02) == 0xFF);
	}
	else if(MyID == 0x03)
	{
		while(Receptor_Ranging(&d03) == 0xFF);
	}
	
	//The second turn
	if(MyID == 0x01)
	{
		while(Initiator_Ranging(0x03) == 0xFF);
		while(Initiator_Ranging(0x02) == 0xFF);
	}
	else if(MyID == 0x02)
	{
		while(Receptor_Ranging(&d12) == 0xFF);
	}
	else if(MyID == 0x03)
	{
		while(Receptor_Ranging(&d13) == 0xFF);
	}
	
	//The third turn
	if(MyID == 0x02)
	{
		while(Initiator_Ranging(0x03) == 0xFF);
	}
	else if(MyID == 0x03)
	{
		while(Receptor_Ranging(&d23) == 0xFF);
	}
	
	//brd turn - 1
	if(MyID == 0x03)
	{
		n3Msg[0] = d03 >> 8;
		n3Msg[1] = (u8)d03;
		n3Msg[2] = d13 >> 8;
		n3Msg[3] = (u8)d13;
		n3Msg[4] = d23 >> 8;
		n3Msg[5] = (u8)d23;
		while(Brd_Msg(n3Msg, 6, 0x00) == 0xFF);
		while(Brd_Msg(n3Msg, 6, 0x01) == 0xFF);
		while(Brd_Msg(n3Msg, 6, 0x02) == 0xFF);	
	}
	else if(MyID == 0x02)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x01)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x00)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	
	//brd turn - 2
	if(MyID == 0x02)
	{
		n2Msg[0] = d02 >> 8;
		n2Msg[1] = (u8)d02;
		n2Msg[2] = d12 >> 8;
		n2Msg[3] = (u8)d12;
		while(Brd_Msg(n2Msg, 4, 0x03) == 0xFF);
		while(Brd_Msg(n2Msg, 4, 0x00) == 0xFF);
		while(Brd_Msg(n2Msg, 4, 0x01) == 0xFF);
	}
	else if(MyID == 0x03)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x01)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x00)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	
	//brd turn - 3
	if(MyID == 0x01)
	{
		n1Msg[0] = d01 >> 8;
		n1Msg[1] = (u8)d01;
		while(Brd_Msg(n1Msg, 2, 0x03) == 0xFF);
		while(Brd_Msg(n1Msg, 2, 0x02) == 0xFF);
		while(Brd_Msg(n1Msg, 2, 0x00) == 0xFF);
	}
	else if(MyID == 0x03)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x02)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	else if(MyID == 0x00)
	{
		while(Receive_BrdMsg(&d01, &d02, &d03, &d12, &d13, &d23) == 0xFF);
	}
	
	//All done, just calculate the coordianates then feedback to upper controller
	n2Axis[1] = d01 >> 8;
	n2Axis[2] = (u8)d01;
	
	temp_calc = (d01 * d01 - d12 * d12 + d02 * d02) / (2 * d01);
	temp_trans = (int16_t)temp_calc;
	x1 = temp_trans;
	
	temp_calc = sqrt(d02 * d02 - temp_calc * temp_calc);
	temp_trans = (int16_t)temp_calc;
	y1 = temp_trans;

	
	temp_calc = (d01 * d01 - d13 * d13 + d03 * d03) / (2 * d01);
	temp_trans = (int16_t)temp_calc;
	x2 = temp_trans;

	
	temp_calc = sqrt(d03 * d03 - temp_calc * temp_calc);
	temp_trans = (int16_t)temp_calc;
	y2 = temp_trans;

	
	comp1 = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	comp1 = sqrt(temp_calc);
	
	
	comp2 = (x1 - x2) * (x1 - x2) + (y1 + y2) * (y1 + y2);
	comp2 = sqrt(temp_calc);
	
	comp1 = comp1 - (double)d23;
	comp1 = comp1 > 0 ? comp1 : -comp1;
	
	comp2 = comp2 - (double)d23;
	comp2 = comp2 > 0 ? comp2 : -comp2;
	
	n3Axis[1] = x1 >> 8;
	n3Axis[2] = (u8)x1;
	n3Axis[3] = y1 >> 8;
	n3Axis[4] = (u8)y1;
	
	n4Axis[1] = x2 >> 8;
	n4Axis[2] = (u8)x2;
		
	if(comp1 < comp2)
	{		
		n4Axis[3] = y2 >> 8;
		n4Axis[4] = (u8)y2;
	}
	else
	{
		y2 = -y2;
		n4Axis[3] = y2 >> 8;
		n4Axis[4] = (u8)y2;
	}
//	
//	//Send my coor
//	for(i = 0; i < 6; i++)
//	{
//		if(MyID == 0x00)
//			USART_SendData(USART2, n1Axis[i]);
//		else if(MyID == 0x01)
//			USART_SendData(USART2, n2Axis[i]);
//		else if(MyID == 0x02)
//			USART_SendData(USART2, n3Axis[i]);
//		else if(MyID == 0x03)
//			USART_SendData(USART2, n4Axis[i]);
//		
//		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
//	}
}
