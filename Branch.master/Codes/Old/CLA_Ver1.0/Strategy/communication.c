/**
  ******************************************************************************
  * ��λ��ͨ��ָ��
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
	//����λ��ͨ�Ÿ�ʽ��0x0A | 0xCF | 1-bit type | 1-bit id | 0xFC
	/*
		ͨ��Э��
		type = 0x01 - ��ָ��id���в��				id = ���Ŀ��id					(����busy����ָ�������Ӧ��ʼ���)
		type = 0x02 - �޸Ĳ��ģʽ 						id = 0x01�رվ�ֵ���
		type = 0x03 - �㲥��Ϣ�������Ӧ      id = Э����id
		type = 0x04 - ָ����Ϣ								id = ��ָ����id         (Ƶ�θ����Լ���Э���㻹������������Զ�ѡ��)
		type = 0x05 - �㲥�Լ�������Ϣ        
		type = 0x06 - ��ʼ��һ�ֵĶ�λ����boss
	*/
	
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++] = ucData;
	
	if (ucRxBuffer[0] != 0x0A)  //��ʼ����У��I�������������¿�ʼѰ��0x0A����ͷ
	{
		ucRxCnt = 0;
		return;
	}
	else if (ucRxCnt == 2 && ucRxBuffer[1] != 0xCF) //��ʼ����У��II - 0xCF
	{
		ucRxCnt = 0;
		return;
	}
	
	if (ucRxCnt < 5) {return;} //���ݲ���5�����򷵻�
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
		if(Usart_RX_flag == SET) //����λ��������Ϣ
		{
				USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, DISABLE);
				Usart_RX_flag = RESET; //clear pending bit
							
				//����ָ��
				if(upperCmd.CmdType == 0x01) //0x01 - ��ָ��id���в��
				{
						if(ADMS == RESET) //��һ�ξ�������ģʽ
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
						else //��5�β����ģʽ
						{
								uint8_t receive_times, accumulator_times = 0, success_times = 0;
								uint16_t dist = 0;
								int32_t temp = 0;
								uint32_t accumulator = 0;
										
								uint16_t dist_buffer[4] = {0, 0, 0, 0};
								uint8_t dist_flag[4] = {0, 0, 0, 0};
											
								//���-max4��
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
											
								//�����˵�������ֵ
								for(receive_times = 0; receive_times < success_times - 1; receive_times++)
								{
										temp = dist_buffer[dist_flag[receive_times]] - dist_buffer[dist_flag[receive_times + 1]];
										if(temp < FTRT)
										{
												accumulator += dist_buffer[dist_flag[receive_times]];
												accumulator_times++;
										}
								}
											
								//������
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
				else if(upperCmd.CmdType == 0x02) //0x02 - �޸Ĳ��ģʽ
				{
					ADMS = (upperCmd.ID == 0x01) ? 0 : 1;
				}
					//����λ��ͨ�Ÿ�ʽ��0x0A | 0xCF | 1-bit type | 1-bit id | 0xFC
					/*
						ͨ��Э��
						type = 0x01 - ��ָ��id���в��				id = ���Ŀ��id					(����busy����ָ�������Ӧ��ʼ���)
						type = 0x02 - �޸Ĳ��ģʽ 						id = 0x01�رվ�ֵ���
						type = 0x03 - �㲥��Ϣ�������Ӧ      id = Э����id
						type = 0x04 - ָ����Ϣ								id = ��ָ����id         (Ƶ�θ����Լ���Э���㻹������������Զ�ѡ��)
						type = 0x05 - �㲥�Լ�������Ϣ    
						type = 0x06 - ��ʼ��һ�ֵĶ�λ����boss
						type = 0x07 - ��Ӧ�㲥��Ϣ
					*/
				else if(upperCmd.CmdType == 0x03) //0x03 - �㲥��Ϣ�������Ӧ
				{
					Broadcast_Msg(upperCmd.ID, 0);
					//�㲥��Ϣ��Ӧ������
					//Waiting_Resp()
				}
				else if(upperCmd.CmdType == 0x04) //0x04 - ָ����Ϣ
				{
					u8 frec = 0xFF;
					
					//�������л�Ƶ��
					switch(myInfo.MyStatus)
					{
						case 0x01: frec = 0; break; //��������
						case 0x02: frec = 1; break; //��������
						default: break;		//����ָ������
					}
					
					if(frec != 0xFF)
					{
						Selected_Msg(upperCmd.ID, frec);
					}
					
					if(myInfo.MyStatus == 0x02) //����չ������࣬�л�Ƶ��
					{
						config.rxCode = 4;
						config.txCode = 4;
						dwt_configure(&config);
						myInfo.RxTx_CodeNUm = 1;
					}				
				}
				else if(upperCmd.CmdType == 0x05) //0x05 - �㲥�Լ�������Ϣ 
				{
					Broadcast_Msg(0xFF, 1);
				}
				else if(upperCmd.CmdType == 0x06) //0x06 - ��ʼ��һ�ֵĶ�λ����boss 
				{
					Boss_Msg();
				}
				else //����ָ��μ�communicationͨ�Žӿ�
				{
									
				}
				USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, ENABLE);
		}
					
		//�������ģʽ
		temp = Receptor_Communication();
		if((uint8_t)temp == 0xAA)
		{
				//���Լ�������ģʽ������λ�����������Ϣ
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
