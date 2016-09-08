/**
  ******************************************************************************
  * 定位的通信指令
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "communication.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FTRT 30 //4-times ranging threshold

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned char ucRxBuffer[5];
RX_Command upperCmd;
__IO uint8_t Usart_RX_flag = RESET;
__IO static uint8_t ADMS = 0; //Accurate Distance Mode Switch
__IO Parse_DW_Data DW_TX_Data;

/* Private function prototypes -----------------------------------------------*/
void DW_TX_Parse(void);

/* Private functions ---------------------------------------------------------*/
void ParseSerialData(unsigned char ucData)
{
	//与上位机通信格式：0x0A | 0xCF | 1-bit type | 1-bit id | 0xFC
	//type = 0x00, id = 0x00是leader
	
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++] = ucData;
	
	if (ucRxBuffer[0] != 0x0A)  //起始数据校验I，若不对则重新开始寻找0x0A数据头
	{
		ucRxCnt = 0;
		return;
	}
	else if (ucRxCnt == 2 && ucRxBuffer[1] != 0xCF) //起始数据校验II - 0xCF
	{
		ucRxCnt = 0;
		return;
	}
	
	if (ucRxCnt < 5) {return;} //数据不满5个，则返回
	else
	{
		memcpy(&upperCmd, &ucRxBuffer[2], 2);
		Usart_RX_flag = SET;
		ucRxCnt = 0;
	}
}

void AI_Stategy(void)
{
	if(Usart_RX_flag == SET) //当上位机发来信息
	{	
		Usart_RX_flag = RESET;
		//处理指令		
		if(upperCmd.CmdType == 0x00)		
		{
			if(upperCmd.ID == 0x00)
			{
				//我是leader，启动对子网的定位
				Leader_Strategy();
			}
		}
	}					
	
	//进入接收模式
	Receptor_Strategy(); //我在监听，根据指令处理
}

void DW_TX_Parse(void)
{
	static uint8_t usart_tx_data[] = {0x0A, 0xCF, 0x00, 0x00, 0x00, 0xFC}; //0x0A | 0xCF | TargetID | 2-bit dist | 0xFC	
	usart_tx_data[2] = DW_TX_Data.TargetID;
	usart_tx_data[3] = (uint8_t)DW_TX_Data.Dist; 				//distance
	usart_tx_data[4] = (uint8_t)(DW_TX_Data.Dist >> 8); //distance
	Usart_TX_SendData(usart_tx_data, sizeof(usart_tx_data));
}
