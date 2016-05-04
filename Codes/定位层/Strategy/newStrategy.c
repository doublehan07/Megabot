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


// 消息类型｜发送者ID｜接受者IDL| 接受IDU｜频段｜坐标x| 坐标y｜成功与否｜丢包与否｜
// 0x    ｜  	｜ st  ｜    |   |   |      |      |       | 0x0D ｜ Ox0A
// Cmtype

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
void SGY_Init_all_id(){
	//...
}

void SGY_Leader(){
	u8 data[MSGLEN]; 
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE) //ÉÏÎ»»ú´ò¶ÏÁË¼àÌý
	{
		upperAsk = 0;
		return;
	}//listening...


	SGY_Recieveing(&data);
	if recieve the indication from master{
		SGY_Init_all_id();
		firstStep_status=0x0;
	}

	if Init is OK{
		//start the four point locating...
		SGY_Indication(locating);	
		data[Cmtype] = Measure;
		data[Sender] = P1;
		data[RecieverL] = P2;
		data[RecieverU] = P4;
		data[Feq] = 0x00;
		data[coodiX] = 0x00;
		data[coodiY] = 0x00;
		data[Success] = FAIL;
		data[Loss] = 0x00;
		SGY_Measuring_Distance(&data);//Measuring P1 with P2 P3 P4 
	}

	if (upperAsk[Cmtype] = Msg && upperAsk[Sender]==P4 && upperAsk[RecieverL] == P3 && upperAsk[RecieverU] == P3 && upperAsk[Success] == SUCCESS){//P4 replies Measuring status by P3 
		data[Cmtype] = secMeasure;
		data[Sender] = P1;
		data[RecieverL] = P2;
		data[RecieverU] = P4;
		data[Feq] = 0x00;
		data[coodiX] = 0x00;
		data[coodiY] = 0x00;
		data[Success] = FAIL;
		data[Loss] = 0x00;
		SGY_Sendmessage(&data);//tell P2 P3 to Start the Second Measure
	}

	if (upperAsk[Cmtype] = secMeasure && upperAsk[Sender] == P2 && upperAsk[RecieverL] == P1 && upperAsk[RecieverU] ==  P1 && upperAsk[Success] == SUCCESS){//
		data[Cmtype] = secMeasure;
		data[Sender] = P1;
		data[RecieverL] = 1;
		data[RecieverU] = n/2;
		data[Feq] = 0x00;
		data[coodiX] = P1X;
		data[coodiY] = P1Y;
		data[Success] = FAIL;
		data[Loss] = 0x00;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			Done[0] = True;
		}
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P4 && upperAsk[RecieverL] == P1 && upperAsk[RecieverU] ==  P1 && upperAsk[Success] == SUCCESS){
		Done[1] = True;
	}

	if (Done[0] && Done[1]){
		//Start the 3rd step
		data[Cmtype] = trdMeasure;
		data[Sender] = P1;
		data[RecieverL] = 1;
		data[RecieverU] = n;
		SGY_Sendmessage();
	}
}

void SGY_P2(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); 
	if(upperAsk == 0xEE) 
	{
		upperAsk = 0;
		return;
	}//listening...
	if (upperAsk[Cmtype] == Measure && upperAsk[RecieveIDL] >=P2 && upperAsk[RecieveIDU] <= P2 && upperAsk[CommCount]==1){
		locatingStatus = passive_Measuring((P2 - upperAsk[Sender] - 1) % 3);
	} 

	if (locatingStatus == True && upperAsk[Cmtype] == Msg && upperAsk[Sender] == P4 && upperAsk[RecieveIDU] >= P2 && upperAsk[RecieveIDU] <= P2 && upperAsk[Success]==SUCCESS){
		data[Cmtype] = Measure;
		data[Sender] = P2;
		data[RecieverL] = P3;
		data[RecieverU] = P4;
		data[Feq] = 0x00;
		data[coodiX] = P2X;
		data[coodiY] = P2Y;
		data[Success] = FAIL;
		SGY_Measuring_Distance(&data);
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] <= P2 && upperAsk[RecieverU] >= P2){
		data[Cmtype] = secMeasure;
		data[Sender] = P2;
		data[RecieverL] = 1;
		data[RecieverU] = n/2;
		data[Feq] = 0x00;
		data[coodiX] = P2X;
		data[coodiY] = P2Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P2;
			data[RecieverL] = P1;
			data[RecieverU] = P1;
			data[Feq] = 0x00;
			data[coodiX] = 0x00;
			data[coodiY] = 0x00;
			data[Success] = FAIL;
			SGY_Sendmessage(&data);
		}
	}
}
void SGY_P3(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE){//
		upperAsk = 0;
		return;
	}//listening...


	if (upperAsk[Cmtype] == Measure && upperAsk[RecieveIDL] >=P3 && upperAsk[RecieveIDU] <= P3 && upperAsk[CommCount]==1){
		passive_Measuring(P3 - upperAsk[Sender] - 1) % 3);
	} 

	if (upperAsk[Cmtype] == Msg && upperAsk[Sender] == P4 && upperAsk[RecieveIDU] >= P3 && upperAsk[RecieveIDU] <= P3 && upperAsk[Success]==SUCCESS){
		channel = ~channel;

		data[Cmtype] = Measure;
		data[Sender] = P3;
		data[RecieverL] = P4;
		data[RecieverU] = P4;
		data[Feq] = 0x00;
		data[coodiX] = 0x00;
		data[coodiY] = 0x00;
		data[Success] = FAIL;
		SGY_Measuring_Distance(&data);
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] <= P3 && upperAsk[RecieverU] >= P3){
		channel=ChB;

		data[Cmtype] = secMeasure;
		data[Sender] = P3;
		data[RecieverL] = n/2+1;
		data[RecieverU] = n;
		data[Feq] = 0x00;
		data[coodiX] = P3X;
		data[coodiY] = P3Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P3;
			data[RecieverL] = P4;
			data[RecieverU] = P4;
			data[Feq] = 0x00;
			data[coodiX] = 0x00;
			data[coodiY] = 0x00;
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);
		}
		else{
			data[Cmtype] = secMeasure;
			data[Sender] = P3;
			data[RecieverL] = P4;
			data[RecieverU] = P4;
			data[Feq] = 0x00;
			data[coodiX] = 0x00;
			data[coodiY] = 0x00;
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);	
		}
		channel = ChA;
	}
}

void SGY_P4(){
	u8 data[MSGLEN];
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE){
		upperAsk = 0;
		return;
	}//listening...


	if (upperAsk[Cmtype] == Measure && upperAsk[RecieveIDL] >=P4 && upperAsk[RecieveIDU] <= P4 && upperAsk[CommCount]==1){
		locatingStatus = passive_Measuring(P4 - upperAsk[Sender] - 1) % 3);
		if (locatingStatus != 0){
			data[Cmtype] = Msg;
			data[Sender] = P4;
			data[RecieverL] = upperAsk[Sender];
			data[RecieverU] = upperAsk[Sender];
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);
		}
	} 

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] <= P4 && upperAsk[RecieverU] >= P4){
		channel = ChB;
	}


	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P3 && upperAsk[RecieverL] <= P4 && upperAsk[RecieverU] >= P4 && upperAsk[Success] == SUCCESS){
		
		data[Cmtype] = secMeasure;
		data[Sender] = P4;
		data[RecieverL] = n/2+1;
		data[RecieverU] = n;
		data[Feq] = 0x00;
		data[coodiX] = P4X;
		data[coodiY] = P4Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P4;
			data[RecieverL] = n/2+1;
			data[RecieverU] = n;
			data[Success] = SUCCESS;			
			SGY_Sendmessage(&data);

			channel = ChA;
			data[Cmtype] = secMeasure;
			data[Sender] = P4;
			data[RecieverL] = P1;
			data[RecieverU] = P1;
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);
		}		
	}
}

void SGY_Receptor(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0);
	if(upperAsk == 0xEE) 
		upperAsk = 0;
		return;
	}
	
	//Listening...
	if Init{
		clear all flag
		delay(number)
		//tell Leader Init is done
		SGY_Sendmessage(&data)
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] ==  P4 && upperAsk[Success] == SUCCESS){
		if (id > n/2){
			channel = ChB;
		}
		else {
			channel = ChA;
		}
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >=id && upperAsk[CommCount]==1){
		msg = passive_Measuring((id-1) % (n/2));
		dist[upperAsk[Sender]] = ;
		//SGY_Calculate_distance()
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P4 && upperAsk[RecieverL] == n/2+1 && upperAsk[RecieverU] == n){
		if (id>upperAsk[RecieverL] && id < upperAsk[RecieverU]){
			channel = ChA;
		}
	} //after secord step set n/2+1 : n in ChA

	if (upperAsk[Cmtype] == trdMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] == 1 && upperAsk[RecieverU] == n){
		channel = ChA;
		if (id == 1){
			data[Cmtype] = trdMeasure;
			data[Sender] = id;
			data[RecieverL] = n/2+1;
			data[RecieverU] = n;
			data[coodiX] = dist[P3];
			data[coodiY] = dist[P4];
			SGY_Measuring_Distance(&data);
		}
		if (id == n/2 + 1){
			data[Cmtype] == trdMeasure;
			data[Sender] == id;
			data[RecieverL] == dist[P1];
			data[RecieverU] == dist[P2];
			SGY_Measuring_Distance(&data);
		}
	}

	if (upperAsk[Cmtype] == trdMeasure && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >=id && upperAsk[CommCount]==1){
		msg = passive_Measuring ((id-1) % (n/2));
		SGY_Calculate_distance(msg);
	}
}


