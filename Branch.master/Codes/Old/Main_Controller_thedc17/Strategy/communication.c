/**
  ******************************************************************************
  * 定位的通信指令
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
	//与下位机通信格式：0x0A | 0xCF | TargetID | 2-bit dist | 0xFC
	
	static u8 ucRxCnt = 0;	
	
	DW_RX_Buffer[ucRxCnt++]=ucData;
	
	if (DW_RX_Buffer[0] != 0x0A)  //起始数据校验I，若不对则重新开始寻找0x0A数据头
	{
		ucRxCnt = 0;
		return;
	}
	else if (ucRxCnt == 2 && DW_RX_Buffer[1] != 0xCF) //起始数据校验II - 0xCF
	{
		ucRxCnt = 0;
		return;
	}
	
	if (ucRxCnt < 6) {return;} //数据不满6个，则返回
	else
	{
		memcpy(&parseData, &DW_RX_Buffer[2], 3);
		ucRxCnt=0;
	}
}
