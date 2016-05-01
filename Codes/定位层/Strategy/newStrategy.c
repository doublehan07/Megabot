#include "newStrategy.h"

// 行为： 初始化
// 	  测距
// 	  指定接受者监听频段
// 	  交换ID
// 	  无
// 消息类型：
// 	  返回某一行为的state
// 	  指令型
// 	  主动型
// 	  被动型

// 丢包判断:
// 	P1,P2,P3,P4的坐标接受错误;
// 	测距错误
// 	id[1:n]的坐标有接受错误
// 	指令错误

// 接受者:
// 	前n个
// 	后n个
// 	全部
// 	某ID

//status 
//	应该能同时包含以下信息：
//	第一次测距成功与否
//	第二次测距成功与否


// 消息类型｜发送者ID｜接受者ID｜行为｜频段｜坐标x| 坐标y｜成功与否｜丢包与否｜
// 0x     ｜  		｜ st   ｜    |    |     |     |        |       | 0x0D ｜ Ox0A

// 0. 选定n-3, n-2, n-1, n为初始点, 四个点之间进行测距//
// 1. 给定四个点绝对位置(P1,P2,P3,P4)，其中P1为Leader节点

// 2. P1对集体广播消息：开始第一步测距, 此时，所有ID将被Init
// 2. P1指定P1,P2监听a频段, 指定P3, P4监听b频段,  
// 3. 对ID进行二分, 由Leader指定前n/2个点使用a频段, 指定后(n+1)/2使用b频段, 并且广播四个点的绝对坐标给所有的ID。所有ID进入监听状态
// 4. ID1在a频段进行广播, 前n/2个点监听, ID1需要等待一定的时间确保所有ID回复, 用一个数组记录每个ID测距成功与否, 
// 5. P2, P3, P4 做同样的操作,完成后发送给P1, 告知其该任务完成，之后翻转频段为a。结果存放于leader的firststate数组中
// 6. P1中的firststate[3]全为true时广播: 开始第二步测距
// 8. ID[n/2+1:n/2]接到第二步测距的指令之后，切换频段为a, 进入监听状态
// 7. ID[2:n/2]接到第二步测距的指令之后, 进入监听状态
// 8. ID1得到第二步测距的指令之后，要广播与ID[n/2+1:n/2]进行测距
// 9. ID[n/2+1:n/2]在得到测距信息之后，计算自己的绝对坐标, 并且将自己的坐标对集体进行广播, 并
// 10. ID[n/2+1]在计算自己的绝对坐标之后, 切换频段为a, 还要广播ID[2:n/2]进行测距
// 11. ID[2:n/2]进行测距之后, 计算自己的绝对坐标, 并且将自己的坐标对集体进行广播
// 12. Leader检查所有ID的坐标, 广播结束此次定位


//Asuming n-3 as leader
void SGY_Four_point_measuring(int total, int ){
	SGY_Sendmessage()
}





void SGY_Internel_measuring(){}
void SGY_Cross_measuring(){}
void SGY_Init_all_id(){}

void SGY_Leader(){
	Listening
	SGY_Recieveing(&data);
	if recieve the indication from master{
		SGY_Init_all_id();
		firstStep_status=0x0;

	}
	if Init is OK{
		//Set P1, P2 in ChA, P3, P4 in CHB
		SGY_Setchannel(P1, ChA);
		SGY_Setchannel(P2, ChA);
		SGY_Setchannel(P3, ChB);
		SGY_Setchannel(P4, ChB);

		//Set ID[1:n/2] in ChA, ID[n/2+1:n] in ChB

		SGY_Setchannel(id[1:n/2], ChA);
		SGY_Setchannel(id[n/2+1:n], ChB);
	
		//Let P3 Measuring the Distance
		SGY_Indication_Measuringdistance(P3, id[1:n/2]);
		//Start Measuring Distance with [1:n/2]
		SGY_Measuring_Distance(&data, id[n/2:n]);

	}

	if (data[recieveID]==P1 && data[action]==firstStep && data[success]==success){//First Step Measuring of P1 is over 
		firstStep_status |= 0x01;
		//Let P2 Mesuring Distance with [1:n/2]	
		SGY_Indication_Measuringdistance(P2);
		SGY_Sendmessage(&data);
	}
	if (data[recieveID]==P2 && data[action]==firstStep && data[success]==success){//First Step Measuring of P2 is over 
		firstStep_status |= 0x02;
	}
	if (data[recieveID]==P3 && data[action]==firstStep && data[success]==success){//First Step Measuring of P3 is over 
		firstStep_status |= 0x04;	
		SGY_Indication_Measuringdistance(P4);
		SGY_Sendmessage(&data);

		//Let P3 stays in CHA 
		SGY_Setchannel(P3, ChA);
	}
	if (data[recieveID]==P1 && data[action]==firstStep && data[success]==success){//First Step Measuring of P4 is over 
		firstStep_status |= 0x08;	
		//Let P4 stays in CHA 
		SGY_Setchannel(P4, ChA);
	}
	if (firstStep_status == 0xF){//First Step is done //status==0xF
		//start second Step Measuring:

		//Let ID[n/2+1:n] stays in ChA
		SGY_Setchannel(id[n/2+1:n], ChA);
		//tell ID[1:n/2] begin the Second Step:// In this step there may be one case that ID[1] has already abandoned \
		SGY_Indication_Measuringdistance(id[1:n/2])										//So if ID[1] recieves then it should boardcast to let other's know \
	}											//so every one should delay a moment to reieve the ok state
	if (data[recieveID]<=n/2 && data[action]==secondStep && data[success]==success){//reply is ok
		//tell that ID to begin the measuring 
		SGY_Indication_Measuringdistance(data[recieveiD], id[n/2+1:n]);
		// and this id should indicate the next measuring
	}
	if (data[recieveID]<=n/2 && data[action]==secondStep && data[success]==success){//Measuring is done//but how to decide the whole processure is done?
		//Set some status 
	}
}

void SGY_P1(){

}
void SGY_P2(){}
void SGY_P3(){}
void SGY_P4(){}

void SGY_Receptor(){
	if Measuring failed:
		status = loss 
	if status == loss{
		return
	}
	//Listening...
	while (!SGY_Recievemessage(&data) ){

	}
	if Init{
		clear all flag
		delay(number)
		//tell Leader Init is done
		SGY_Sendmessage(&data)
	}
	if passive measuring of first step{
		//SGY_Calculate_distance()
	}
	if First ID recieve the indication of start the second step measuring{
		SGY_Measuring_Distance()
	}
	if n-th ID recieve {
		//measuring the Distance between ID[n/2] and ID2:n/2] 
		SGY_Measuring_Distance()
	}
	if passive measuring of second step{
		SGY_Calculate_distance()
	}

}

