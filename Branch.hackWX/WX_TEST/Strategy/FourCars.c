///**
//  ******************************************************************************
//  * 
//  ******************************************************************************
//  */

///* Includes ------------------------------------------------------------------*/
//#include "FourCars.h"
//#include "ranging_api.h"

//#include <math.h>

///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//Node ourNodes[4];
//static double Treply1, Tround2;
//static double Tround1, Treply2;
//static double distanceM;
//static u16 distanceCM;

///* Private function prototypes -----------------------------------------------*/
//static void Node_RangeOnce(u8 Target1, u8 Target2, u8 Target3);
//static void Get_Axis(void);

///* Private functions ---------------------------------------------------------*/
//void Range_Once(void) //���ò�ຯ������ʼһ�ֶ�λ
//{
//	u8 counterRank = 0;
//	u8 myTurnFlag = 0;
//	//��ʼһ�ֹ㲥���
//	do
//	{
//		switch (counterRank)
//		{
//			case 0: 
//				if (MyID == 0x00)
//					myTurnFlag = 1;
//				break;
//			case 1:
//				if (MyID == 0x01)
//					myTurnFlag = 2;
//				break;
//			case 2:
//				if (MyID == 0x02)
//					myTurnFlag = 3;
//				break;
//			case 3:
//				if (MyID == 0x03)
//					myTurnFlag = 4;
//				break;
//			default: break;
//		}
//		
//		if (myTurnFlag)
//		{
//			switch (myTurnFlag)
//			{
//				case 1: Node_RangeOnce(0x01, 0x02, 0x03); break;
//				case 2: Node_RangeOnce(0x00, 0x02, 0x03); break;
//				case 3: Node_RangeOnce(0x00, 0x01, 0x03); break;
//				case 4: Node_RangeOnce(0x00, 0x01, 0x02); break;
//				default: break; //bug
//			}
//			myTurnFlag = 0;
//			counterRank++;
//		}
//		else
//		{
//			u8 tempTarget;
//			u16 tempSend;
//			
//			while (Safe_Receive(1000000));
//			
//			//�յ���Ϣ�Ĵ���
//			if (rx_buffer[0] == 0xF1) //�յ���poll_msg��Ӧ����
//			{
//				if(rx_buffer[2] <= MyID && MyID < rx_buffer[3]) //����ͨ�ŷ�Χ��
//				{
//					//���½ڵ���
//					tempTarget = rx_buffer[1];
//					ourNodes[tempTarget].ID = rx_buffer[1];
//					
//					//��������
//					ourNodes[tempTarget].X = ((u16)rx_buffer[5] << 8) | (u16)rx_buffer[4];
//					ourNodes[tempTarget].Y = ((u16)rx_buffer[7] << 8) | (u16)rx_buffer[6];
//					
//					//���½���ʱ���
//					rx_DeviceB_1 = get_rx_timestamp_u64();
//					
//					//��Ӧresp_msg
//					Resp_Msg(tempTarget);
//				}
//				else; //�Ҳ���ͨ�ŷ�Χ��
//			}
//			else if (rx_buffer[0] == 0xF3) //�յ���final_msg
//			{
//				if (rx_buffer[1] == tempTarget) //��鷢�����ǲ���poll-target
//				{
//					if (tempTarget == 0x00)
//					{
//						switch (MyID)
//						{
//							case 0x00: break; //bug
//							case 0x01: 
//								tempSend = rx_buffer[2] * 100 + rx_buffer[3];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[4] * 100 + rx_buffer[5];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x02: 
//								tempSend = rx_buffer[6] * 100 + rx_buffer[7];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[8] * 100 + rx_buffer[9];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x03: 
//								tempSend = rx_buffer[10] * 100 + rx_buffer[11];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[12] * 100 + rx_buffer[13];
//								Treply2 = tempSend / 1000.0;
//								break;
//							default: break; //bug
//						}
//					}
//					else if (tempTarget == 0x01)
//					{
//						switch (MyID)
//						{
//							case 0x00: 
//								tempSend = rx_buffer[2] * 100 + rx_buffer[3];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[4] * 100 + rx_buffer[5];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x01: break; //bug
//							case 0x02: 
//								tempSend = rx_buffer[6] * 100 + rx_buffer[7];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[8] * 100 + rx_buffer[9];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x03: 
//								tempSend = rx_buffer[10] * 100 + rx_buffer[11];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[12] * 100 + rx_buffer[13];
//								Treply2 = tempSend / 1000.0;
//								break;
//							default: break; //bug
//						}
//					}
//					else if (tempTarget == 0x02)
//					{
//						switch (MyID)
//						{
//							case 0x00: 
//								tempSend = rx_buffer[2] * 100 + rx_buffer[3];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[4] * 100 + rx_buffer[5];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x01: 
//								tempSend = rx_buffer[6] * 100 + rx_buffer[7];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[8] * 100 + rx_buffer[9];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x02: break; //bug
//							case 0x03: 
//								tempSend = rx_buffer[10] * 100 + rx_buffer[11];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[12] * 100 + rx_buffer[13];
//								Treply2 = tempSend / 1000.0;
//								break;
//							default: break; //bug
//						}
//					}
//					else if (tempTarget == 0x03)
//					{
//						switch (MyID)
//						{
//							case 0x00: 
//								tempSend = rx_buffer[2] * 100 + rx_buffer[3];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[4] * 100 + rx_buffer[5];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x01: 
//								tempSend = rx_buffer[6] * 100 + rx_buffer[7];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[8] * 100 + rx_buffer[9];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x02:
//								tempSend = rx_buffer[10] * 100 + rx_buffer[11];
//								Tround1 = tempSend / 1000.0;
//								tempSend = rx_buffer[12] * 100 + rx_buffer[13];
//								Treply2 = tempSend / 1000.0;
//								break;
//							case 0x03: break; //bug
//							default: break; //bug
//						}
//					}
//					rx_DeviceB_3 = get_rx_timestamp_u64();
//				
//					Treply1 = (tx_DeviceB_2 - rx_DeviceB_1) * TIM_DIS_UNIT;
//					Tround2 = (rx_DeviceB_3 - tx_DeviceB_2) * TIM_DIS_UNIT;
//					
//					distanceM = (Treply1 * Treply2 - Tround2 * Tround1) / (Treply1 + Treply2 + Tround1 + Tround2);
//					distanceM = distanceM > 0 ? distanceM : -distanceM;
//					distanceCM = (u16)(distanceM * 100.0 + 0.5);
//					
//					//���¾�����Ϣ
//					ourNodes[tempTarget].Dist[MyID] = distanceCM;
//					
//					counterRank++;
//				}
//				else; //�����߲���
//			}
//		}
//	}
//	while (counterRank <= 4);
//	
//	//�㲥����������ʼ���ι㲥�Լ��Ĳ������
//	counterRank = 0;
//	myTurnFlag = 0;
//	do
//	{
//		switch (counterRank)
//		{
//			case 0: 
//				if (MyID == 0x00)
//					myTurnFlag = 1;
//				break;
//			case 1:
//				if (MyID == 0x01)
//					myTurnFlag = 2;
//				break;
//			case 2:
//				if (MyID == 0x02)
//					myTurnFlag = 3;
//				break;
//			case 3:
//				if (MyID == 0x03)
//					myTurnFlag = 4;
//				break;
//			default: break;
//		}
//		
//		//Brd_Msg
//		//CmdType | SenderID | dist1a | dist1b | dist2a | dist2b | dist3a | dist3b | CRC | CRC
//		if (myTurnFlag)
//		{
//			myTurnFlag = 0;
//			Brd_Msg();
//			counterRank++;
//		}
//		else
//		{
//			while (Safe_Receive(1000000));
//			
//			//�յ���Ϣ�Ĵ���
//			if (rx_buffer[0] == 0xB0) //�յ��˹㲥��Ϣ
//			{
//				u16 tempDist;
//			
//				tempDist = (u16)rx_buffer[3] << 8 | (u16)rx_buffer[2];
//				ourNodes[rx_buffer[1]].Dist[0] = tempDist;
//				tempDist = (u16)rx_buffer[5] << 8 | (u16)rx_buffer[4];
//				ourNodes[rx_buffer[1]].Dist[1] = tempDist;
//				tempDist = (u16)rx_buffer[7] << 8 | (u16)rx_buffer[6];
//				ourNodes[rx_buffer[1]].Dist[2] = tempDist;
//				
//			}
//			else; //�յ���������Ϣ������
//		}
//	}
//	while (counterRank <= 4);
//	
//	//���ݾ����������
//	Get_Axis();
//}

//static void Node_RangeOnce(u8 Target1, u8 Target2, u8 Target3)
//{
//	//����һ�ι㲥���
//	//Target1, Target2, Target3
//	Poll_Msg (0x00, 0x04, 0, 0); //����ɹ����Ͳ�����˷���ʱ�������Ҫת����
//	if (Safe_Receive(100) == 0) //�ȴ���һ����Ӧ
//	{
//		if (rx_buffer[0] == 0xF2) //У��Resp_Msg
//		{
//			if (rx_buffer[1] == Target1)
//				rx_DeviceA_2a = get_rx_timestamp_u64();
//			else if (rx_buffer[1] == Target2)
//				rx_DeviceA_2b = get_rx_timestamp_u64();
//			else if (rx_buffer[1] == Target3)
//				rx_DeviceA_2c = get_rx_timestamp_u64();
//			else
//				return;
//			
//			if (Safe_Receive(100) == 0) //�ȴ��ڶ�����Ӧ
//			{
//				if (rx_buffer[0] == 0xF2) //У��Resp_Msg
//				{
//					if (rx_buffer[1] == Target1)
//						rx_DeviceA_2a = get_rx_timestamp_u64();
//					else if (rx_buffer[1] == Target2)
//						rx_DeviceA_2b = get_rx_timestamp_u64();
//					else if (rx_buffer[1] == Target3)
//						rx_DeviceA_2c = get_rx_timestamp_u64();
//					else
//						return;
//					
//					if (Safe_Receive(100) == 0) //�ȴ���������Ӧ
//					{
//						if (rx_buffer[0] == 0xF2) //У��Resp_Msg
//						{
//							if (rx_buffer[1] == Target1)
//								rx_DeviceA_2a = get_rx_timestamp_u64();
//							else if (rx_buffer[1] == Target2)
//								rx_DeviceA_2b = get_rx_timestamp_u64();
//							else if (rx_buffer[1] == Target3)
//								rx_DeviceA_2c = get_rx_timestamp_u64();
//							else
//								return;
//							
//							//����ˣ���ʼfinal_Msg
//							Final_Msg(1000, tx_DeviceA_1, rx_DeviceA_2a, rx_DeviceA_2b, rx_DeviceA_2c);
//							return;
//						}
//						else; //������У��ʧ��
//					}
//					else; //100ms֮�ڵ�������Ӧû������
//				}
//				else; //�ڶ���У��ʧ��
//			}
//			else; //100ms֮�ڵڶ�����Ӧû������
//		}
//		else; //��һ��У��ʧ��
//	}
//	else; //100ms֮�ڵ�һ����Ӧû��������bug�ˡ�
//}

//static void Get_Axis(void)
//{
//	static u8 i, saveI;
//	static int16_t x1, y1, x2, y2, x3, y3, x4, y4;
//	static int16_t y3a, y3b, y4a, y4b, min;
//	static u16 d12, d23, d13, d24, d14, d34;
//	static int32_t tempCal[4];
//	
//	d12 = ourNodes[0].Dist[0];
//	d23 = ourNodes[1].Dist[1];
//	d13 = ourNodes[0].Dist[1];
//	d24 = ourNodes[1].Dist[2];
//	d14 = ourNodes[0].Dist[2];
//	d34 = ourNodes[2].Dist[2];
//	
//	x1 = 0;
//	y1 = 0;
//	x2 = d12;
//	y2 = 0;
//	x3 = (d12 * d12 - d23 * d23 + d13 * d13) * 1.0 / 2.0 / d12;
//	x4 = (d12 * d12 - d23 * d24 + d14 * d14) * 1.0 / 2.0 / d12;
//	y3a = sqrt(d13 * d13 - x3 * x3);
//	y3b = -y3a;
//	y4a = sqrt(d13 * d14 - x4 * x4);
//	y4b = -y4a;
//	
//	tempCal[0] = d14 * d14 - d34 * d34 - 2 * x3 * x4 - 2 * y3a * y4a;
//	tempCal[1] = d14 * d14 - d34 * d34 - 2 * x3 * x4 - 2 * y3b * y4b;
//	tempCal[2] = d14 * d14 - d34 * d34 - 2 * x3 * x4 - 2 * y3a * y4b;
//	tempCal[3] = d14 * d14 - d34 * d34 - 2 * x3 * x4 - 2 * y3b * y4a;
//	
//	min = 0xFFFF;
//	for (i = 0; i < 4; i++)
//	{
//		if (min > tempCal[i])
//		{
//			min = tempCal[i];
//			saveI = i;
//		}
//	}
//	
//	switch(saveI)
//	{
//		case 0: y3 = y3a; y4 = y4a; break;
//		case 1: y3 = y3b; y4 = y4b; break;
//		case 2: y3 = y3a; y4 = y4b; break;
//		case 3: y3 = y3b; y4 = y4a; break;
//	}
//	
//	ourNodes[0].X = x1; ourNodes[0].Y = y1;
//	ourNodes[1].X = x2; ourNodes[1].Y = y2;
//	ourNodes[2].X = x3; ourNodes[2].Y = y3;
//	ourNodes[3].X = x4; ourNodes[3].Y = y4;
//}
