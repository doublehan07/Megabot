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
void Leader_Strategy(void) //我是leader，我要启动子网的定位
{
	static u8 Resp_ID[3] = {0, 0, 0};
	static double Axis[2] = {0, 0};
	//更新我的info状态
	MyInfo_Init(Axis, Axis, 0x05);
	//我开始启动广播，在我网络里面的点都要回应，同时记录下我的坐标0,0
	NetInfo_Init(MyID, 0, 0); //我也记录我的坐标
	Broadcast_Msg(0xFF, 0, Resp_ID);
	
	if(Resp_ID[0] == 2) //正常来说，我会收到两个人的回应，取最前面那个好了
	{
		//我开始和最近的点测距，同时确定坐标系
		Initiator_Ranging(Resp_ID[1], 1, 0x01, 0);
		//我测距的时候，要告诉下一个点主动权交给它啦
	}
}

void Receptor_Strategy(void) //我是监听者，我要继续子网的定位
{
	static u8 upperAsk = 0;
	do
	{
		upperAsk = Receptor_Listening();
	}
	while(upperAsk == 0); //等待接收到消息
	if(upperAsk == 0xAA) //上位机打断了监听
	{
		upperAsk = 0;
		return;
	}
	
	//接收到的消息放在rx_buffer里面
	//第一种可能，收到某个点的广播消息，若未翻转状态必须回应！
	if(rx_buffer[0] == 0x03) //广播消息
	{
		static u16 RectX, RectY;
		//首先，我要记录下这个广播消息的内容
		RectX = ((u16)rx_buffer[4] << 8) & (u16)rx_buffer[3];
		RectY = ((u16)rx_buffer[6] << 8) & (u16)rx_buffer[5];
		NetInfo_Init(rx_buffer[2], RectX, RectY);
		
		//如果我不是被指定的协作点，而且我没有翻转状态，只管回复就好了
		if(rx_buffer[1] != MyID && myInfo.MyStatus == 0x00)
		{
			//要不要delay一段时间再回复？
			Resp_Msg();
		}
		else if(rx_buffer[1] == MyID)//我是被指定的协作点
		{
			//Coordianator_Strategy()
			//测距是怎么工作的呢
			//首先，测距发起节点广播自己的坐标，同时指定了协作节点
			//其它状态没翻转的点通通回应！
			//发起者和协作者暗戳戳地记录下自己听到的前两个点的id
			//发起者再次指定第一个id，告诉他我们不翻转频段测距
			//协作者听到这个id，记下来，指定另一个id，告诉他我们翻转频段测距
			//另一个id于是翻转了频段
			//发起者听到了协作点的指令，于是他开始了测距
			//他们开始测距，测距的时候要告诉两个新的点，这是第几次测距，所以他们就能记录距离信息计算了
			//第几次测距，测距点的身份(status)，这次测距完成之后是否需要切换频段(0-不切换，其余对应频段代号)
			//测距之后，发起者和协作者翻转频段，再次对目标测距
			//如果一组完成而另一组没完成怎么办呢？若测距请求发出后没回应，监听一下有没有人在说话，若有等待，若无再试一次
			//再测一组距离，同时告诉发起者指定的新节点要翻转频段
			//新节点对另一个节点测距，新节点发起。冲突请求监听同上
			//有三个距离，计算自己的信息，广播一下
			//第二个新节点广播，开始下一组测距
			//先不要考虑三次里面有一次fail的情况啦QwQ
		}
	}
	else if(rx_buffer[0] == 0x01) //测距消息，我被指定测距啦，测距就好，反正我也不知道发生了什么
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
