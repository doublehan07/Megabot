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
	MyInfo_Init(Axis, Axis, 0x05);
	//�ҿ�ʼ�����㲥��������������ĵ㶼Ҫ��Ӧ��ͬʱ��¼���ҵ�����0,0
	NetInfo_Init(MyID, 0, 0); //��Ҳ��¼�ҵ�����
	Broadcast_Msg(0xFF, 0, Resp_ID);
	
	if(Resp_ID[0] == 2) //������˵���һ��յ������˵Ļ�Ӧ��ȡ��ǰ���Ǹ�����
	{
		//�ҿ�ʼ������ĵ��࣬ͬʱȷ������ϵ
		Initiator_Ranging(Resp_ID[1], 1, 0x01, 0);
		//�Ҳ���ʱ��Ҫ������һ��������Ȩ��������
	}
}

void Receptor_Strategy(void) //���Ǽ����ߣ���Ҫ���������Ķ�λ
{
	static u8 upperAsk = 0;
	do
	{
		upperAsk = Receptor_Listening();
	}
	while(upperAsk == 0); //�ȴ����յ���Ϣ
	if(upperAsk == 0xAA) //��λ������˼���
	{
		upperAsk = 0;
		return;
	}
	
	//���յ�����Ϣ����rx_buffer����
	//��һ�ֿ��ܣ��յ�ĳ����Ĺ㲥��Ϣ����δ��ת״̬�����Ӧ��
	if(rx_buffer[0] == 0x03) //�㲥��Ϣ
	{
		static u16 RectX, RectY;
		//���ȣ���Ҫ��¼������㲥��Ϣ������
		RectX = ((u16)rx_buffer[4] << 8) & (u16)rx_buffer[3];
		RectY = ((u16)rx_buffer[6] << 8) & (u16)rx_buffer[5];
		NetInfo_Init(rx_buffer[2], RectX, RectY);
		
		//����Ҳ��Ǳ�ָ����Э���㣬������û�з�ת״̬��ֻ�ܻظ��ͺ���
		if(rx_buffer[1] != MyID && myInfo.MyStatus == 0x00)
		{
			//Ҫ��Ҫdelayһ��ʱ���ٻظ���
			Resp_Msg();
		}
		else if(rx_buffer[1] == MyID)//���Ǳ�ָ����Э����
		{
			//Coordianator_Strategy()
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
	else if(rx_buffer[0] == 0x01) //�����Ϣ���ұ�ָ������������ͺã�������Ҳ��֪��������ʲô
	{
		
	}
}

void NetInfo_Init(u8 ID, u16 RectX, u16 RectY)
{
	if(netCnt > netInfoSIZE) {return;}
	netInfo[netCnt].ID = ID;
	netInfo[netCnt].RectX = RectX;
	netInfo[netCnt].RectY = RectY;
	netCnt++;
}	

void MyInfo_Init(double Rect_Axis[2], double Polar_Axis[2], u8 MyStatus)
{
	myInfo.Rect_Axis[0] = Rect_Axis[0];
	myInfo.Rect_Axis[1] = Rect_Axis[1];
	myInfo.Polar_Axis[0] = Polar_Axis[0];
	myInfo.Polar_Axis[1] = Polar_Axis[1];
	myInfo.MyStatus = MyStatus;
}
