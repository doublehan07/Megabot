/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "strategy.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO MyInfo myInfo = {{0,0}, {0,0}, MyID, 0x00};
NetInfo netInfo[netInfoSIZE];
__IO u8 netCnt = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Leader_Strategy(void) //����leader����Ҫ���������Ķ�λ
{
	static u8 Resp_ID[3] = {0, 0, 0};
	static double Axis[2] = {0, 0};
	//�����ҵ�info״̬
	MyInfo_Init(Axis, Axis, 0xFF);
	//�ҿ�ʼ�����㲥��������������ĵ㶼Ҫ��Ӧ��ͬʱ��¼���ҵ�����0,0
	NetInfo_Init(MyID, 0, 0); //��Ҳ��¼�ҵ�����
	Broadcast_Msg(0xFF, Resp_ID);
	
	if(Resp_ID[0] >= 1) //������˵���һ��յ������˵Ļ�Ӧ��ȡ��ǰ���Ǹ�����
	{
		//�ҿ�ʼ������ĵ��࣬ͬʱȷ������ϵ
		Initiator_Ranging(Resp_ID[1], 1, 0xFF, 0);
		//�Ҳ���ʱ��Ҫ������һ��������Ȩ��������
	}
}

void Receptor_Strategy(void) //���Ǽ����ߣ���Ҫ���������Ķ�λ
{
	static u8 upperAsk = 0;
	do
	{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //�ȴ����յ���Ϣ
	if(upperAsk == 0xEE) //��λ������˼���
	{
		upperAsk = 0;
		return;
	}
	
	//���յ�����Ϣ����rx_buffer����
	//��һ�ֿ��ܣ��յ�ĳ���� �Ĺ㲥��Ϣ����δ��ת״̬�����Ӧ��
	if(rx_buffer[0] == 0x03) //�㲥��Ϣ
	{
		static int16 RectX, RectY;
		//���ȣ���Ҫ��¼������㲥��Ϣ������
		RectX = ((int16)rx_buffer[4] << 8) | (int16)rx_buffer[3];
		RectY = ((int16)rx_buffer[6] << 8) | (int16)rx_buffer[5];
		NetInfo_Init(rx_buffer[2], RectX, RectY);
		
		//����Ҳ��Ǳ�ָ����Э���㣬������û�з�ת״̬��ֻ�ܻظ��ͺ���
		if(rx_buffer[1] != MyID && myInfo.MyStatus == 0x00)
		{
			//Ҫ��Ҫdelayһ��ʱ���ٻظ���
			Resp_Msg();
		}
		else if(rx_buffer[1] == MyID)//���Ǳ�ָ����Э����
		{
			Coordianator_Strategy(rx_buffer[2]);
			//�������ô��������
			//���ȣ���෢��ڵ�㲥�Լ������꣬ͬʱָ����Э���ڵ�
			//����״̬û��ת�ĵ�ͨͨ��Ӧ��
			//�����ߺ�Э���߰������ؼ�¼���Լ�������ǰ�������id
			//�������ٴ�ָ����һ��id�����������ǲ���תƵ�β��
			//Э�����������id����������ָ����һ��id�����������Ƿ�תƵ�β��
			//��һ��id���Ƿ�ת��Ƶ��
			//������������Э�����ָ���������ʼ�˲��
			//���ǿ�ʼ��࣬����ʱ��Ҫ���������µĵ㣬���ǵڼ��β�࣬�������Ǿ��ܼ�¼������Ϣ������
			//�ڼ��β�࣬��������(status)����β�����֮���Ƿ���Ҫ�л�Ƶ��(0-���л��������ӦƵ�δ���)
			//���֮�󣬷����ߺ�Э���߷�תƵ�Σ��ٴζ�Ŀ����
			//���һ����ɶ���һ��û�����ô���أ���������󷢳���û��Ӧ������һ����û������˵�������еȴ�����������һ��
			//�ٲ�һ����룬ͬʱ���߷�����ָ�����½ڵ�Ҫ��תƵ��
			//�½ڵ����һ���ڵ��࣬�½ڵ㷢�𡣳�ͻ�������ͬ��
			//���������룬�����Լ�����Ϣ���㲥һ��
			//�ڶ����½ڵ�㲥����ʼ��һ����
			//�Ȳ�Ҫ��������������һ��fail�������QwQ
		}
	}
	else if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[3] == 0x01) //�����Ϣ���ұ�ָ������������ͺã�������Ҳ��֪��������ʲô
	{
		Ranging_Strategy();
		//֪����leader�ڵ㷢�Ĳ����������ֱ�ӽ�ϵ��
		//����ϵҲҪ���߱�ĵ����ǵ�����ϵ
		//����Ǳ�Ĳ��������С���Ӱ����������������׼��������
	}
	else if(rx_buffer[0] == 0x04 && rx_buffer[1] == MyID) //ָ����Ϣ���ұ�ָ������
	{
		//�ұ�ָ��֮��Ҫ��ʲô�أ�����ָ���ҵĻ��˵ı�ţ�֮�����ʱ�����ѡ�����ˣ�
		Change_Freq(rx_buffer[3]);		
	}
	else if(rx_buffer[0] == 0x05) //�㲥ָ�����������Ϣ
	{
		static int16 RectX, RectY;
		//�Ҽ�¼������㲥��Ϣ�����ݼ���
		RectX = ((int16)rx_buffer[3] << 8) | (int16)rx_buffer[2];
		RectY = ((int16)rx_buffer[5] << 8) | (int16)rx_buffer[4];
		NetInfo_Init(rx_buffer[1], RectX, RectY);
	}
	else if(rx_buffer[0] == 0x08) //Last_One_Msg()
	{
		if(rx_buffer[1] == MyID)
		{
			LastOne_Strategy();
		}
	}
}

void Ranging_Strategy(void)
{
	u8 flag, SourceID, Times, UpperStatus, which_freq;
	
	u16 dist_Array[3] = {0, 0, 0};
	u8 id_Array[3] = {0, 0, 0};
	
	do
	{
		flag = Receptor_Ranging(&SourceID, &Times, &UpperStatus, &which_freq);
	}
	while(flag != 0xAA);

	if(Times == 1)
	{
		if(UpperStatus == 0x01)
		{
			myInfo.MyStatus = 0x03;
		}
		else if(UpperStatus == 0x02)
		{
			myInfo.MyStatus = 0x04;
		}
	}
	
	//�ɹ���࣬����Ҫ���������Ϣ��
	if(UpperStatus == 0xFF) //leader����Ϣ����ϵ
	{
		//�����ҵ�info״̬
//		myInfo.Polar_Axis[0] = distance_cm;
		myInfo.Rect_Axis[0] = distance_cm;
		myInfo.MyStatus = 0x05;
		//��֪������ͨ�Ž����ˣ�������Ҫ�㲥��Ϣ��ʼ��һ��ͨ��
		Change_Freq(0);
		FirstOne_Strategy(SourceID);
	}
	else if(UpperStatus == 0x01) //�����������ͨ����
	{
		dist_Array[Times - 1] = distance_cm;
		id_Array[Times - 1] = SourceID;
	}
	else if(UpperStatus == 0x02) //�����������ͨ����
	{
		u8 CorpID;
		int16 Axis[4];
		dist_Array[Times - 1] = distance_cm;
		id_Array[Times - 1] = SourceID;
		CorpID = which_freq;
		
		if(myInfo.MyStatus == 0x03 && Times == 2) //�����β�࣬�ҵ�����
		{
			u8 i, counter = 0;
			int16 tempa[2], tempb[2];
			Change_Freq(1);
			//�����ҿ��ܵ�λ�ã�����u16��Ȼ�󷢹�ȥ˳�����
			//���ȵõ�ǰ�����������
			for(i = 0; i < netCnt; i++)
			{
				if(netInfo[i].ID == id_Array[0])
				{
					tempa[0] = netInfo[i].RectX;
					tempa[1] = netInfo[i].RectY;
					counter++;
				}
				else if(netInfo[i].ID == id_Array[1])
				{
					tempb[0] = netInfo[i].RectX;
					tempb[1] = netInfo[i].RectY;
					counter++;
				}
				if(counter >= 2)
				{
					break;
				}
			}
			//Ȼ����������ҵĿ���λ��
			Calculate_My_Pos(tempa, tempb, dist_Array, Axis);
			if(Corp_Ranging(CorpID, 3, 0x03, 1, Axis) == 0x0A) //�ɹ����������Ҫ�л��س�ʼƵ��
			{
				dist_Array[2] = distance_cm;
				Change_Freq(0);
				//û��ʲô���ˣ��ҵ���Э����㲥�ҵ���������Լ�������
				myInfo.MyStatus = 0x05;
			}
		}
	}	
	else if(UpperStatus == 0x03) //������ָ����λ�ýڵ����ͨ����
	{
		u8 i, counter = 0;
		int16 tempa[2], tempb[2], myAxis[4], corpAxis[4];
		dist_Array[2] = distance_cm;
		id_Array[2] = SourceID;
		Change_Freq(0);
		//����һ���ҵ�����
		//���ȵõ�ǰ�����������
		for(i = 0; i < netCnt; i++)
		{
			if(netInfo[i].ID == id_Array[0])
			{
				tempa[0] = netInfo[i].RectX;
				tempa[1] = netInfo[i].RectY;
				counter++;
			}
			else if(netInfo[i].ID == id_Array[1])
			{
				tempb[0] = netInfo[i].RectX;
				tempb[1] = netInfo[i].RectY;
				counter++;
			}
			if(counter >= 2)
			{
				break;
			}
		}
		//Ȼ��õ��ҿ��ܵ���������
		Calculate_My_Pos(tempa, tempb, dist_Array, myAxis);
		//�õ�������Ϣ�жԷ����ܵ���������
		corpAxis[0] = (u16)rx_buffer[16] | ((u16)rx_buffer[17] << 8);
		corpAxis[1] = (u16)rx_buffer[18] | ((u16)rx_buffer[19] << 8);
		corpAxis[2] = (u16)rx_buffer[20] | ((u16)rx_buffer[21] << 8);
		corpAxis[3] = (u16)rx_buffer[22] | ((u16)rx_buffer[23] << 8);
		
		//Ȼ��������һ�ξ�����Ϣ�õ����մ�
		Decide_Our_Pos(myAxis, corpAxis, dist_Array[2], SourceID);
		
		FirstOne_Strategy(SourceID);
	}
	else if(UpperStatus == 0xFE) //���һ������Ҳ��
	{
		return;
	}
	else //�ţ�������������ţ�
	{
		
	}
}

void FirstOne_Strategy(u8 CorpID)
{
	u8 flag = 0, corp_TarID;
	u8 Resp_ID[3] = {0, 0, 0};
	myInfo.MyStatus = 0x01; //BUG�����㲥��Ϣ�������δ֪�Ĵ���
	//�ҿ�ʼ�����㲥��������������ĵ㶼Ҫ��Ӧ��ͬʱ��¼���ҵ�����(RectX, RectY)
	NetInfo_Init(MyID, myInfo.Rect_Axis[0], myInfo.Rect_Axis[1]); //��Ҳ��¼�ҵ�����
	Delay(7);
	Broadcast_Msg(CorpID, Resp_ID);
	
	if(Resp_ID[0] == 2) //�����ҳɹ��յ���������Ӧ
	{
		//�����һ�ָ����һ���յ��Ľڵ㣬�������ϴ����л�Ƶ��hhh
		Delay(1);
		Selected_Msg(Resp_ID[1], 0);
	}
	else if(Resp_ID[0] == 1) //�������ã�ֻʣһ���ڵ���Ҫ��λ��
	{
		//��������������ֻ��������֪�ڵ㣬�޷���������Э��
		//��ʱ�������ͨ�ţ����������Σ�����������ת
		//��ָ�������㣬���������������꣬������ֱ���������ͺ�
		//�������������֪�ڵ㣬�Һ�Э����֮�⣬������һ���㣬���㶨λ
		//����й�netCnt��if-elseֱ���õ������Լ��жϾ��У���ֻ��Msg������
		Delay(1);
		Last_One_Msg(Resp_ID[1]);
		myInfo.MyStatus = 0x05;
		return;
	}
	else if(Resp_ID[0] == 0) //���֣�û�е��ˣ�������λ�ͺ�
	{
		//�һ��Ƿ�һ����Ϣ��Э�������˰ɣ���ȻЭ����Ҫ��bug
		Delay(1);
		Stop_Waiting_Msg();
		myInfo.MyStatus = 0x05;
		return;
		//������һ���رս��չ��ܵĽӿڣ���ʡ�����ã�Ȼ�������ò���
		//Low_Power_Interface()
	}
	else //���������
	{
		//Error_Handler()
	}
	//�ҿ�ʼ����Э�����ѡ����Ϣ����������ʽ������Ҳ����timeout
	flag = Receptor_Listening(0);
	if(flag == 0x01)
	{
		if(rx_buffer[0] == 0x04 && rx_buffer[2] == CorpID)
		{
			corp_TarID = rx_buffer[1];
		}
		else
		{
			//��һ��Э�����֪�Լ�û�����Ľӿڣ������Ȳ�����
		}
	}
	else if(flag == 0xEE) //����λ�����
	{
		myInfo.MyStatus = 0x05;
		return;
	}
	else
	{
		//Error_Handler()
	}
	
	//��֪��Э����Ҳ֪��������ʼ��һ�ֲ��ɣ��е�С����hhh
	if(Initiator_Ranging(Resp_ID[1], 1, 0x01, 0) == 0xAA) //�ɹ����������Ҫ�л�Ƶ�� BUG������λ����ϸĳ�0xEE
	{
		Change_Freq(1);
		//����Ҫ��ʧ�ܵļ���������ƣ���ʱ��ûд
		//Listening_Handler()
		//��ʼ�ڶ��ֲ�֮࣬�������ͺ����޹���
		if(Initiator_Ranging(corp_TarID, 2, 0x01, 0) == 0xAA)
		{
			Change_Freq(0);
			myInfo.MyStatus = 0x05;
			return;
		}
	}
	myInfo.MyStatus = 0x05;
}

void Coordianator_Strategy(u8 boss_ID)
{
	u8 flag = 0, boss_TarID, myTarID;
	u8 Resp_ID[3] = {0, 0, 0};
	//�ұ�ָ��Э���ˣ�����ת˫��״̬��
	Double_Buff_Recp_Listening(Resp_ID, 0);
	myInfo.MyStatus = 0x02;
	//�ҿ�ʼ�����������ѡ����Ϣ����������ʽ������Ҳ����timeout
	flag = Receptor_Listening(5000);
	if(flag == 0x01)
	{
		if(rx_buffer[0] == 0x04 && rx_buffer[2] == boss_ID)
		{
			boss_TarID = rx_buffer[1];
		}
		else if(rx_buffer[0] == 0x08) //Last_One_Msg()����ֱ�Ӹ���MyStatus+return�ͺ�
		{
			myInfo.MyStatus = 0x05;
			return;
		}
		else if(rx_buffer[0] == 0x09) //Stop_Waiting_Msg()����ֱ�Ӹ���MyStatus+return�ͺ�
		{
			myInfo.MyStatus = 0x05;
			return;	
		}
		else //�յ��˱����Ϣ
		{
			//Error_Handler()
		}
	}
	else if(flag == 0xEE) //����λ�����
	{
		myInfo.MyStatus = 0x05;
		return;
	}
	else //���մ���
	{
		//Error_Handler()
	}
	
	if(Resp_ID[0] == 2) //�����ҳɹ��յ���������Ӧ
	{
		if(Resp_ID[1] == boss_TarID)
		{
			myTarID = Resp_ID[2];			
		}
		else
		{
			myTarID = Resp_ID[1];
		}
		
		Selected_Msg(myTarID, 1);		
		Change_Freq(1);
		//��ʼ��һ�ֲ��ɣ���Ҳ�е�С����hhh
		if(Initiator_Ranging(myTarID, 1, 0x02, 0) == 0xAA) //�ɹ����������Ҫ�л�Ƶ��
		{
			Change_Freq(0);
			//����Ҫ��ʧ�ܵļ���������ƣ���ʱ��ûд
			//Listening_Handler()
			if(Initiator_Ranging(boss_TarID, 2, 0x02, myTarID) == 0xAA)
			{
				//Ҳû��ɶ����
				myInfo.MyStatus = 0x05;
				return;
			}
		}
	}
	else
	{
		//��ʱ���п������ҵ�ͨ�ų�������
		//Error_Handler()
	}
	myInfo.MyStatus = 0x05;
}

void LastOne_Strategy(void)
{
	//��������������ֻ��������֪�ڵ㣬�޷���������Э��
	//��ʱ�������ͨ�ţ����������Σ�����������ת
	//�������������֪�ڵ㣬�Һ�Э����֮�⣬������һ���㣬���㶨λ
	//����й�netCnt��if-elseֱ���õ������Լ��жϾ���
	if(netCnt == 2)
	{
		int16 Axis[4] = {0, 0, 0, 0};
		u16 dist_Array[2] = {0, 0};
		int16 posA[2] = {0, 0};
		int16 posB[2] = {0, 0};
		
		posA[0] = netInfo[0].RectX;
		posA[1] = netInfo[0].RectY;
		posB[0] = netInfo[1].RectX;
		posB[1] = netInfo[1].RectY;
		
		if(Initiator_Ranging(netInfo[0].ID, 1, 0xFE, 0) == 0xAA)
		{
			dist_Array[0] = distance_cm;
			if(Initiator_Ranging(netInfo[1].ID, 1, 0xFE, 0) == 0xAA)
			{
				dist_Array[1] = distance_cm;
				Calculate_My_Pos(posA, posB, dist_Array, Axis);
				myInfo.MyStatus = 0x05;
				myInfo.Rect_Axis[0] = Axis[0];
				myInfo.Rect_Axis[1] = Axis[1];
				//��Ҳ��¼���ҵ���Ϣ
				NetInfo_Init(MyID, Axis[0], Axis[1]);
				//���ߴ���ҵ���Ϣ
				CorpInfo_Msg(MyID, Axis);
			}
		}		
	}
	else if(netCnt > 2)
	{
		int16 Axis[2] = {0, 0};
		u16 dist_Array[3] = {0, 0, 0};
		int16 posA[2] = {0, 0};
		int16 posB[2] = {0, 0};
		int16 posC[2] = {0, 0};
		
		posA[0] = netInfo[netCnt - 1].RectX;
		posA[1] = netInfo[netCnt - 1].RectY;
		posB[0] = netInfo[netCnt - 2].RectX;
		posB[1] = netInfo[netCnt - 2].RectY;
		posC[0] = netInfo[netCnt - 3].RectX;
		posC[1] = netInfo[netCnt - 3].RectY;
		
		if(Initiator_Ranging(netInfo[netCnt - 1].ID, 1, 0xFE, 0) == 0xAA)
		{
			dist_Array[0] = distance_cm;
			if(Initiator_Ranging(netInfo[netCnt - 2].ID, 1, 0xFE, 0) == 0xAA)
			{
				dist_Array[1] = distance_cm;
				if(Initiator_Ranging(netInfo[netCnt - 3].ID, 1, 0xFE, 0) == 0xAA)
				{
					dist_Array[2] = distance_cm;
					//�ɹ��õ����ݣ������ҵ�����
					Cal_Mypos_Triangle(posA, posB, posC, dist_Array, Axis);
					myInfo.MyStatus = 0x05;
					myInfo.Rect_Axis[0] = Axis[0];
					myInfo.Rect_Axis[1] = Axis[1];
					//��Ҳ��¼���ҵ���Ϣ
					NetInfo_Init(MyID, Axis[0], Axis[1]);
					//���ߴ���ҵ���Ϣ
					CorpInfo_Msg(MyID, Axis);
				}
			}
		}
	}
	else
	{
		//Error_Handler()
	}
}

void NetInfo_Init(u8 ID, int16 RectX, int16 RectY)
{
	if(netCnt > netInfoSIZE) {return;}
	netInfo[netCnt].ID = ID;
	netInfo[netCnt].RectX = RectX;
	netInfo[netCnt].RectY = RectY;
	netCnt++;
}	

void MyInfo_Init(double Rect_Axis[2], double Polar_Axis[2], u8 MyStatus)
{
	myInfo.Rect_Axis[0] = (int16)Rect_Axis[0];
	myInfo.Rect_Axis[1] = (int16)Rect_Axis[1];
	myInfo.Polar_Axis[0] = (int16)Polar_Axis[0];
	myInfo.Polar_Axis[1] = (int16)Polar_Axis[1];
	myInfo.MyStatus = MyStatus;
}

void Change_Freq(u8 flag)
{
	dwt_forcetrxoff(); //Must be set in IDLE mode.
	if(flag == 1)
	{
		config.rxCode = 4;
		config.txCode = 4;				
	}
	else
	{
		config.rxCode = 3;
		config.txCode = 3;				
	}
	dwt_configure(&config);
}

void Calculate_My_Pos(int16 *tempa, int16 *tempb, u16 *dist_Array, int16 *Axis)
{
	double x1 = tempa[0], y1 = tempa[1], x2 = tempb[0], y2 = tempb[1], d1 = dist_Array[0], d2 = dist_Array[1];
	double xa = 0.0, ya = 0.0, xb = 0.0, yb = 0.0;
	
	if(tempa[0] == tempb[0]) //A = 0
	{
		double B, C, D, E, F, G;
		B = 2 * (y1 - y2);
		C = x1 * x1 - x2 * x2 + y1 * y1 - y2 * y2 - d1 * d1 + d2 * d2;
		ya = yb = C / B;
		D = ya - y1;
		E = 2 * x1;
		F = x1 * x1 + D * D - d1 * d1;
		G = E * E - 4 * F;
		xa = (sqrt(G) + E) / 2;
		xb = (-sqrt(G) + E) / 2;
	}
	else //A != 0
	{
		double A, B, C, D, E, F, G, H;
		A = 2 * (x1 - x2);
		B = 2 * (y1 - y2);
		C = x1 * x1 - x2 * x2 + y1 * y1 - y2 * y2 - d1 * d1 + d2 * d2;
		C = C / A;
		D = B / A;
		E = D * D + 1.0;
		F = 2 * x1 * D - 2 * C * D - 2 * y1;
		G = C * C + x1 * x1 - 2 * x1 * C + y1 * y1 - d1 * d1;
		H = F * F - 4 * E * G ;
		ya = (sqrt(H) - F) / (2 * E);
		yb = (-sqrt(H) - F) / (2 * E);
		xa = C - D * ya;
		xb = C - D * yb;
	}
	
	Axis[0] = (int16)xa;
	Axis[1] = (int16)ya;
	Axis[2] = (int16)xb;
	Axis[3] = (int16)yb;
}

void Cal_Mypos_Triangle(int16 posA[2], int16 posB[2], int16 posC[2], u16 dist_Array[3], int16 Axis[2])
{
	double A, B, C, D, E, F;
	int16 Axis_temp[4];
	Calculate_My_Pos(posA, posB, dist_Array, Axis_temp);
	
	A = posC[0] - Axis_temp[0];
	B = posC[1] - Axis_temp[1];
	C = A * A + B * B - dist_Array[2] * dist_Array[2];
	C = C > 0 ? C : -C;
	
	D = posC[0] - Axis_temp[2];
	E = posC[1] - Axis_temp[3];
	F = D * D + E * E - dist_Array[2] * dist_Array[2];
	F = F > 0 ? F : -F;
	
	if(C < F)
	{
		Axis[0] = Axis_temp[0];
		Axis[1] = Axis_temp[1];
	}
	else
	{
		Axis[0] = Axis_temp[2];
		Axis[1] = Axis_temp[3];
	}
}

void Decide_Our_Pos(int16 *myAxis, int16 *corpAxis, u16 dist, u8 CorpID)
{
	u8 i, j;
	double myx, myy, corpx, corpy;
	double a, b, discal;
	double res = dist;
	res = res * res;
	for(i = 0; i < 3; i += 2)
	{
		for(j = 0; j < 3; j += 2)
		{
			myx = myAxis[i]; myy = myAxis[i+1];
			corpx = corpAxis[j]; corpy = corpAxis[j+1];
			a = myx - corpx; b = myy - corpy;
			discal = a * a + b * b;
			if(discal - res <= 1.0) //����
			{
				break;
			}
		}
	}
	CorpInfo_Msg(CorpID, corpAxis + j);
	myInfo.Rect_Axis[0] = myAxis[i];
	myInfo.Rect_Axis[1] = myAxis[i+1];
	myInfo.MyStatus = 0x05;
}
