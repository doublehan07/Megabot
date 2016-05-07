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
	/*
		通信协议
		type = 0x01 - 与指定id进行测距				id = 测距目标id					(若不busy，被指定必须回应开始测距)
		type = 0x02 - 修改测距模式 						id = 0x01关闭均值输出
		type = 0x03 - 广播消息，必须回应      id = 协作点id
		type = 0x04 - 指定消息								id = 被指定点id         (频段根据自己是协作点还是主动点身份自动选择)
		type = 0x05 - 广播自己坐标消息        
		type = 0x06 - 开始新一轮的定位，大boss
	*/
	
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

void Ranging_Stategy(void)
{
		uint16_t temp = 0;
		if(Usart_RX_flag == SET) //当上位机发来信息
		{
				USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, DISABLE);
				Usart_RX_flag = RESET; //clear pending bit
							
				//处理指令
				if(upperCmd.CmdType == 0x01) //0x01 - 与指定id进行测距
				{
						if(ADMS == RESET) //测一次距离就输出模式
						{
								uint8_t counter_receive_times;
								uint8_t get_dist_flag = RESET;
								for(counter_receive_times = 0; counter_receive_times < 5; counter_receive_times++)
								{
										if(Initiator_Communication(upperCmd.ID))
										{
												get_dist_flag = SET;
												break;
										}
								}
								if(get_dist_flag)
								{
										DW_TX_Data.TargetID = upperCmd.ID;							//TargetID
										DW_TX_Data.Dist = distance_cm;									//distance
										DW_TX_Parse();
								}
								else
								{
										DW_TX_Data.TargetID = upperCmd.ID;			//TargetID
										DW_TX_Data.Dist = 0xFFFF; 							//error: 0xFFFF										
										DW_TX_Parse();
								}
						}
						else //测5次才输出模式
						{
								uint8_t receive_times, accumulator_times = 0, success_times = 0;
								uint16_t dist = 0;
								int32_t temp = 0;
								uint32_t accumulator = 0;
										
								uint16_t dist_buffer[4] = {0, 0, 0, 0};
								uint8_t dist_flag[4] = {0, 0, 0, 0};
											
								//测距-max4次
								for(receive_times = 0; receive_times < 4; receive_times++)
								{
										if(Initiator_Communication(upperCmd.ID))
										{
												dist_buffer[receive_times] = distance_cm;
												dist_flag[success_times] = receive_times;
												success_times++;
												Delay(1);
										}
								}
											
								//做差滤掉非正常值
								for(receive_times = 0; receive_times < success_times - 1; receive_times++)
								{
										temp = dist_buffer[dist_flag[receive_times]] - dist_buffer[dist_flag[receive_times + 1]];
										if(temp < FTRT)
										{
												accumulator += dist_buffer[dist_flag[receive_times]];
												accumulator_times++;
										}
								}
											
								//输出结果
								if(accumulator_times == 0)
								{
										DW_TX_Data.TargetID = upperCmd.ID;			//TargetID
										DW_TX_Data.Dist = 0xFFFF; 							//error: 0xFFFF										
										DW_TX_Parse();
								}
								else
								{
										dist = accumulator / accumulator_times;
												
										temp = dist_buffer[dist_flag[success_times - 1]] - dist;
										temp = temp > 0 ? temp : -temp;
										if(temp < FTRT)
										{
												dist = (accumulator + dist_buffer[dist_flag[success_times - 1]]) / (accumulator_times + 1);
										}
										
										DW_TX_Data.TargetID = upperCmd.ID;							//TargetID
										DW_TX_Data.Dist = dist;													//distance													
										DW_TX_Parse();
								}								
						}
				}
				else if(upperCmd.CmdType == 0x02) //0x02 - 修改测距模式
				{
					ADMS = (upperCmd.ID == 0x01) ? 0 : 1;
				}
					//与上位机通信格式：0x0A | 0xCF | 1-bit type | 1-bit id | 0xFC
					/*
						通信协议
						type = 0x01 - 与指定id进行测距				id = 测距目标id					(若不busy，被指定必须回应开始测距)
						type = 0x02 - 修改测距模式 						id = 0x01关闭均值输出
						type = 0x03 - 广播消息，必须回应      id = 协作点id
						type = 0x04 - 指定消息								id = 被指定点id         (频段根据自己是协作点还是主动点身份自动选择)
						type = 0x05 - 广播自己坐标消息    
						type = 0x06 - 开始新一轮的定位，大boss
						type = 0x07 - 回应广播消息
					*/
				else if(upperCmd.CmdType == 0x03) //0x03 - 广播消息，必须回应
				{
					Broadcast_Msg(upperCmd.ID, 0);
					//广播消息回应处理函数
					//Waiting_Resp()
				}
				else if(upperCmd.CmdType == 0x04) //0x04 - 指定消息
				{
					u8 frec = 0xFF;
					
					//被动点切换频段
					switch(myInfo.MyStatus)
					{
						case 0x01: frec = 0; break; //主动测距点
						case 0x02: frec = 1; break; //被动测距点
						default: break;		//无需指定命令
					}
					
					if(frec != 0xFF)
					{
						Selected_Msg(upperCmd.ID, frec);
					}
					
					if(myInfo.MyStatus == 0x02) //若开展被动测距，切换频段
					{
						config.rxCode = 4;
						config.txCode = 4;
						dwt_configure(&config);
						myInfo.RxTx_CodeNUm = 1;
					}				
				}
				else if(upperCmd.CmdType == 0x05) //0x05 - 广播自己坐标消息 
				{
					Broadcast_Msg(0xFF, 1);
				}
				else if(upperCmd.CmdType == 0x06) //0x06 - 开始新一轮的定位，大boss 
				{
					Boss_Msg();
				}
				else //其他指令，参见communication通信接口
				{
									
				}
				USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, ENABLE);
		}
					
		//进入接收模式
		temp = Receptor_Communication();
		if((uint8_t)temp == 0xAA)
		{
				//若自己进入测距模式，向上位机传输距离信息
				myInfo.MyStatus = 0x05;
				DW_TX_Data.TargetID = (temp >> 8);							//TargetID
				DW_TX_Data.Dist = distance_cm;									//distance						
				DW_TX_Parse();
		}
			
		/* Debug purpose. Test for ranging speed. */
//		dwt_setGPIOvalue(GDM3, GDP3);
//		Delay(100); //100ms
//		dwt_setGPIOvalue(GDM3, 0);
}

void DW_TX_Parse(void)
{
	static uint8_t usart_tx_data[] = {0x0A, 0xCF, 0x00, 0x00, 0x00, 0xFC}; //0x0A | 0xCF | TargetID | 2-bit dist | 0xFC	
	usart_tx_data[2] = DW_TX_Data.TargetID;
	usart_tx_data[3] = (uint8_t)DW_TX_Data.Dist; 				//distance
	usart_tx_data[4] = (uint8_t)(DW_TX_Data.Dist >> 8); //distance
	Usart_TX_SendData(usart_tx_data, sizeof(usart_tx_data));
}
