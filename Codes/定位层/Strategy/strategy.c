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
	MyInfo_Init(Axis, Axis, 0xFF);
	//我开始启动广播，在我网络里面的点都要回应，同时记录下我的坐标0,0
	NetInfo_Init(MyID, 0, 0); //我也记录我的坐标
	Broadcast_Msg(0xFF, Resp_ID);
	
	if(Resp_ID[0] == 2) //正常来说，我会收到两个人的回应，取最前面那个好了
	{
		//我开始和最近的点测距，同时确定坐标系
		Initiator_Ranging(Resp_ID[1], 1, 0xFF, 0);
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
			Coordianator_Strategy(rx_buffer[2]);
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
	else if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[3] == 0x01) //测距消息，我被指定测距啦，测距就好，反正我也不知道发生了什么
	{
		Ranging_Strategy();
		//知道是leader节点发的测距请求，我们直接建系！
		//建完系也要告诉别的点我们的坐标系
		//如果是别的测距请求，拿小本子把三个距离记下来，准备算坐标
	}
	else if(rx_buffer[0] == 0x04 && rx_buffer[1] == MyID) //指定信息，我被指定啦！
	{
		//我被指定之后要做什么呢？记下指定我的坏人的编号？之后测距的时候就有选择性了？
		Change_Freq(rx_buffer[3]);		
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
	
	//成功测距，现在要处理这个信息了
	if(UpperStatus == 0xFF) //leader的消息，建系
	{
		//更新我的info状态
		myInfo.Polar_Axis[0] = distance_cm;
		myInfo.Rect_Axis[0] = distance_cm;
		myInfo.MyStatus = 0x05;
		//我知道这组通信结束了，于是我要广播消息开始下一组通信
		Change_Freq(0);
		FirstOne_Strategy(SourceID);
	}
	else if(UpperStatus == 0x01) //主动测距点和我通信啦
	{
		dist_Array[Times - 1] = distance_cm;
		id_Array[Times - 1] = SourceID;
	}
	else if(UpperStatus == 0x02) //被动测距点和我通信啦
	{
		u8 CorpID;
		u16 Axis[4];
		dist_Array[Times - 1] = distance_cm;
		id_Array[Times - 1] = SourceID;
		CorpID = which_freq;
		
		if(myInfo.MyStatus == 0x03 && Times == 2) //第三次测距，我的主场
		{
			u8 i, counter = 0;
			u16 tempa[2], tempb[2];
			Change_Freq(1);
			//计算我可能的位置，两组u16，然后发过去顺带测距
			//首先得到前两个点的坐标
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
			//然后计算两个我的可能位置
			Calculate_My_Pos(tempa, tempb, dist_Array, Axis);
			if(Corp_Ranging(CorpID, 3, 0x03, 1, Axis) == 0x0A) //成功测距啦，我要切换回初始频段
			{
				dist_Array[2] = distance_cm;
				Change_Freq(0);
				//没我什么事了，我等着协作点广播我的坐标和它自己的坐标
			}
		}
	}	
	else if(UpperStatus == 0x03) //主动点指定的位置节点和我通信啦
	{
		u8 i, counter = 0;
		u16 tempa[2], tempb[2], myAxis[4], corpAxis[4];
		dist_Array[2] = distance_cm;
		id_Array[2] = SourceID;
		Change_Freq(0);
		//计算一下我的坐标
		//首先得到前两个点的坐标
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
		//然后得到我可能的两个坐标
		Calculate_My_Pos(tempa, tempb, dist_Array, myAxis);
		//得到这条信息中对方可能的两个坐标
		corpAxis[0] = (u16)rx_buffer[16] | ((u16)rx_buffer[17] << 8);
		corpAxis[1] = (u16)rx_buffer[18] | ((u16)rx_buffer[19] << 8);
		corpAxis[2] = (u16)rx_buffer[20] | ((u16)rx_buffer[21] << 8);
		corpAxis[3] = (u16)rx_buffer[22] | ((u16)rx_buffer[23] << 8);
		
		//然后根据最后一次距离信息得到最终答案
		Decide_Our_Pos(myAxis, corpAxis, dist_Array[2], SourceID);
		
		FirstOne_Strategy(SourceID);
	}
	else //嗯？例外情况？空着！
	{
		
	}
}

void FirstOne_Strategy(u8 CorpID)
{
	u8 flag = 0, corp_TarID;
	u8 Resp_ID[3] = {0, 0, 0};
	myInfo.MyStatus = 0x01;
	//我开始启动广播，在我网络里面的点都要回应，同时记录下我的坐标(RectX, RectY)
	NetInfo_Init(MyID, myInfo.Rect_Axis[0], myInfo.Rect_Axis[1]); //我也记录我的坐标
	Broadcast_Msg(CorpID, Resp_ID);
	if(Resp_ID[0] == 2) //假设我成功收到了两个回应
	{
		//于是我会指定第一个收到的节点，咱们是老大不用切换频段hhh
		Selected_Msg(Resp_ID[1], 0);
	}
	else
	{
		
	}
	//我开始监听协作点的选择信息
	do
	{
		flag = Receptor_Listening();
		if(flag && rx_buffer[0] == 0x04 && rx_buffer[2] == CorpID)
		{
			corp_TarID = rx_buffer[1];
			break;
		}
		else
		{
			flag = 0;
		}
	}
	while(flag != 1);
	//我知道协作点也知道啦，开始第一轮测距吧！有点小紧张hhh
	if(Initiator_Ranging(Resp_ID[1], 1, 0x01, 0) == 0xAA) //成功测距啦，我要切换频段
	{
		Change_Freq(1);
		//这里要有失败的监听处理机制，暂时还没写
		//Listening_Handler()
		//开始第二轮测距，之后的事情就和我无关啦
		if(Initiator_Ranging(corp_TarID, 2, 0x01, 0) == 0xAA)
		{
			Change_Freq(0);
		}
	}
	
}

void Coordianator_Strategy(u8 boss_ID)
{
	u8 flag = 0, boss_TarID, myTarID;
	u8 Resp_ID[3] = {0, 0, 0};
	//我被指定协作了，马上转双收状态！
	Double_Buff_Recp_Listening(Resp_ID, 0);
	myInfo.MyStatus = 0x02;
	//我开始监听主动点的选择信息
	do
	{
		flag = Receptor_Listening();
		if(flag && rx_buffer[0] == 0x04 && rx_buffer[2] == boss_ID)
		{
			boss_TarID = rx_buffer[1];
			break;
		}
		else
		{
			flag = 0;
		}
	}
	while(flag != 1);
	if(Resp_ID[0] == 2) //假设我成功收到了两个回应
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
		//开始第一轮测距吧！我也有点小紧张hhh
		if(Initiator_Ranging(myTarID, 1, 0x02, 0) == 0xAA) //成功测距啦，我要切换频段
		{
			Change_Freq(0);
			//这里要有失败的监听处理机制，暂时还没写
			//Listening_Handler()
			if(Initiator_Ranging(boss_TarID, 2, 0x02, myTarID) == 0xAA)
			{
				//也没我啥事了
			}
		}
	}
	else
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

void Calculate_My_Pos(u16 *tempa, u16 *tempb, u16 *dist_Array, u16 *Axis)
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
	
	Axis[0] = (u16)xa;
	Axis[1] = (u16)ya;
	Axis[2] = (u16)xb;
	Axis[3] = (u16)yb;
}

void Decide_Our_Pos(u16 *myAxis, u16 *corpAxis, u16 dist, u8 CorpID)
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
			if(discal - res <= 1.0) //对了
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
