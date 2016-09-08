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
//   | ID | RecieveID | RecieveID |  
//
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
void SGY_Leader(){
	
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE) //ÉÏÎ»»ú´ò¶ÏÁË¼àÌý
	{
		upperAsk = 0;
		return;
	}//listening...
	if recieve the indication from master{
		Done[0] = 0;
		Done[1] = 0;
		firstStatus = 0;
		secondStatus = 0;
		thirdStatus = 0;
	}

	if Init is ok {
		//start the four point locating...
		data[Cmtype] = fstMeasure;
		data[Sender] = P1;
		data[RecieverL] = P2;
		data[RecieverU] = P4;
		SGY_Measuring_Distance(&data);//Measuring P1 with P2 P3 P4 
	}

	if (upperAsk[Cmtype] = fstMeasure && upperAsk[Sender]==P4 && upperAsk[RecieverL] == P3 && upperAsk[RecieverU] == P3){//P4 replies Measuring status by P3 
		data[Cmtype] = secMeasure;
		data[Sender] = P1;
		data[RecieverL] = P2;
		data[RecieverU] = P4;
		SGY_Sendmessage(&data);//tell P2 P3 to Start the Second Measure
	}

	if (upperAsk[Cmtype] = secMeasure && upperAsk[Sender] == P2 && upperAsk[RecieverL] == P1 && upperAsk[RecieverU] == P1){//
		data[Cmtype] = secMeasure;
		data[Sender] = P1;
		data[RecieverL] = 1;
		data[RecieverU] = n/2;
		data[coodiX] = P1X;
		data[coodiY] = P1Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			Done[0] = True;
		}
	}

	if (upperAsk[Cmtype] == fstMeasure && upperAsk[RecieveIDL] >=P2 && upperAsk[RecieveIDU] <= P2 && upperAsk[CommCount]==1){//first state measuring
		locatingStatus = passive_Measuring((P2 - upperAsk[Sender] - 1) % 3);
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
		SGY_Sendmessage(&data);
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
	if (upperAsk[Cmtype] == fstMeasure && upperAsk[Sender] == P1 && upperAsk[RecieveIDL] ==P2 && upperAsk[RecieveIDU] == P2 && upperAsk[CommCount]==1){
		dist[upperAsk[Sender] - n] = passive_Measuring((P2 - upperAsk[Sender] - 1) % 3);//只有dist[1]因为只有P1会发送测距信息给P2，所以P2得到的是P1和｝P2 的距离
	} 

	if (locatingStatus == True && upperAsk[Cmtype] == Msg && upperAsk[Sender] == P4 && upperAsk[RecieveIDU] >= P2 && upperAsk[RecieveIDU] <= P2 && upperAsk[Success]==SUCCESS){
		data[Cmtype] = fstMeasure;
		data[Sender] = P2;
		data[RecieverL] = P3;
		data[RecieverU] = P4;
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
		data[coodiX] = P2X;
		data[coodiY] = P2Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P2;
			data[RecieverL] = P1;
			data[RecieverU] = P1;
			data[coodiX] = 0x00;
			data[coodiY] = 0x00;
			data[Success] = FAIL;
			SGY_Sendmessage(&data);
		}
	}
	
	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P4 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P2 && upperAsk[Success] == SUCCESS){
		Done[1] = True;
	}

	if (Done[0] && Done[1]){
		//Start the 3rd step
		data[Cmtype] = trdMeasure;
		data[Sender] = P1;
		data[RecieverL] = 1;
		data[RecieverU] = n;
		SGY_Sendmessage(&data);
	}
}

void SGY_P3(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //
	if(upperAsk == 0xEE){//
		upperAsk = 0;
		return;
	}//listening...


	if (upperAsk[Cmtype] == fstMeasure && upperAsk[RecieveIDL] >=P3 && upperAsk[RecieveIDU] <= P3 && upperAsk[CommCount]==1){
		passive_Measuring(P3 - upperAsk[Sender] - 1) % 3);
	} 

	if (upperAsk[Cmtype] == Msg && upperAsk[Sender] == P4 && upperAsk[RecieveIDU] >= P3 && upperAsk[RecieveIDU] <= P3 && upperAsk[Success]==SUCCESS){
		channel = ~channel;

		data[Cmtype] = Measure;
		data[Sender] = P3;
		data[RecieverL] = P4;
		data[RecieverU] = P4;
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
		data[coodiX] = P3X;
		data[coodiY] = P3Y;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P3;
			data[RecieverL] = P4;
			data[RecieverU] = P4;
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


	if (upperAsk[Cmtype] == fstMeasure && upperAsk[RecieveIDL] >=P4 && upperAsk[RecieveIDU] <= P4 && upperAsk[CommCount]==1){//first step measuring(passive measuring)
		dist = passive_Measuring((P4 - upperAsk[Sender] - 1) % 3)
		if (dist != 0){
			date[Cmtype] = MSG;
			data[action] = fstMeasure;
			data[Sender] = P4;
			data[RecieverL] = upperAsk[Sender]+1;
			data[RecieverU] = upperAsk[Sender]+1;
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);

			//P4 should save the distance with P1 and P2 
		}
	} 

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] <= P4 && upperAsk[RecieverU] >= P4){
		channel = ChB;
	}


	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P3 && upperAsk[RecieverL] <= P4 && upperAsk[RecieverU] >= P4 && upperAsk[Success] == SUCCESS){
		//start the second step: measuring the distances with n/2+1:n and P4    Message is secMeasure P4 n/2+1 n P4X P4Y 0 0 CommCount=1 FAIL
		data[Cmtype] = secMeasure;
		data[Sender] = P4;
		data[RecieverL] = n/2+1;
		data[RecieverU] = n;
		data[coodiX] = P4X;
		data[coodiY] = P4Y;
		data[CommCount] = 1;
		data[Success] = FAIL;
		secondStatus = SGY_Measuring_Distance(&data);//send measuring message to measure with n/2+1:n
		if (secondStatus){
			data[Cmtype] = secMeasure;
			data[Sender] = P4;
			data[RecieverL] = n/2+1;
			data[RecieverU] = n;
			data[CommCount] = 0;
			data[Success] = SUCCESS;			
			SGY_Sendmessage(&data);				//this part is to tell n/2+1:n to set in ChA! Because P4 is going to set in ChA, and before ChA is set, these  

			channel = ChA;                       //tell P1 that the second step measure is done. Because P4 was in ChB when measuring n/2+1:n, 											 
			data[Cmtype] = secMeasure;			//so ChA should be set before connecting with P1
			data[Sender] = P4;
			data[RecieverL] = P1;
			data[RecieverU] = P1;
			data[CommCount] = 0;
			data[Success] = SUCCESS;
			SGY_Sendmessage(&data);
		}		
	}
}

void SGY_Receptor(){
	//variances list
	int x;
	int y;
	int x1,x2,y1,y2;
	int P1X, P1Y, P2X, P2Y, P3X, P3Y, P4X, P4Y;
	u8 upperAsk[]

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

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P1 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P4 && upperAsk[Success] == SUCCESS){
		if (id > n/2)
			channel = ChB;
		else 
			channel = ChA;
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] >= P1 && upperAsk[Sender] <= P4 && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >=id && upperAsk[CommCount] == 1){   //passive secMeasure  
		msg = passive_Measuring((id-1) % (n/2));           																		//the message is secMeasure P1-P4 1 n 0 0 CommCount==1 0
		if (id < n/2 + 1){																										
			if (upperAsk[Sender] == P1){
				dist1 = msg[Dist1];
				P1X = msg[coodiX];
				P1Y = msg[coodiY];
			}
			if (upperAsk[Sender] == P2){
				dist2 = msg[Dist2];
				P2X = msg[coodiX];
				P2Y = msg[coodiY];
			}
			if (dist1 != 0 && dist2 != 0){           									//if dist1 and dist2 are attained
				d = sqrt((P1X - P2X)*(P1X - P2X) + (P1Y - P2Y)*(P1Y - P2Y));
				p = (dist1 + dist2 + d) / 2;
				S = sqrt(p*(p-dist1)*(p-dist2)*(p-d));
				h = 2 * S / d;
				a = sqrt(dist1*dist1 - h * h);
				x_ = dist1 / d * P2X + (1 - dist1 / d) * P1X;
				y_ = dist2 / d * P2Y + (1 - dist1 / d) * P1Y;
				x1 = x_ + (P2X - P1X) * h / d;
				y1 = y_ - (P2Y - P1Y) * h / d;
				x2 = x_ - (P2X - P1X) * h / d;
				y2 = y_ + (P2Y - P1Y) * h / d;
			}

		}
		else {

			if (upperAsk[Sender] == P3){
				dist1 = msg[Dist1];
				P3X = msg[coodiX];
				P3Y = msg[coodiY];
			}
			if (upperAsk[Sender] == P4){
				dist2 = msg[Dist2];
				P4X = msg[coodiX];
				P4Y = msg[coodiY];
			}
			if (dist1 != 0 && dist2 != 0){												//if dist1 and dist2 are attained
				d = sqrt((P3X - P4X)*(P3X - P4X) + (P3Y - P4Y)*(P3Y - P4Y));
				p = (dist1 + dist2 + d) / 2;
				S = sqrt(p*(p-dist1)*(p-dist2)*(p-d));
				h = 2 * S / d;
				a = sqrt(dist1*dist1 - h * h);
				x_ = dist1 / d * P4X + (1 - dist1 / d) * P3X;
				y_ = dist2 / d * P4Y + (1 - dist1 / d) * P3Y;
				x1 = x_ + (P4X - P3X) * h / d;
				y1 = y_ - (P4Y - P3Y) * h / d;
				x2 = x_ - (P4X - P3X) * h / d;
				y2 = y_ + (P4Y - P3Y) * h / d;
			}
		}

		//Calculate_distance()
	}

	if (upperAsk[Cmtype] == secMeasure && upperAsk[Sender] == P4 && upperAsk[RecieverL] == n/2+1 && upperAsk[RecieverU] == n &&upperAsk[CommCount] == 0){
		if (id>upperAsk[RecieverL] && id < upperAsk[RecieverU]){			      		//this part aims to set 
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
			data[coodiX] = x1;
			data[coodiY] = y1;
			data[Dist1] = x2;
			data[Dist2] = y2;

			//send x1, x2, y1, y2
			SGY_Measuring_Distance(&data);
		}
		if (id == n/2 + 1){
			data[Cmtype] == trdMeasure;
			data[Sender] == id;
			data[RecieverL] == 1;
			data[RecieverU] == n/2;
			data[coodiX] = x1;
			data[coodiY] = y1;
			data[Dist1] = x2;
			data[Dist2] = y2;

			//send x1, x2, y1, y2;
			SGY_Measuring_Distance(&data);
		}
	}

	if (upperAsk[Cmtype] == trdMeasure && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >=id && upperAsk[CommCount] == 1){
		//data[coodiX] represent X1 cooridnate of Sender (if reciever's id is in 1:n/2 then Sender is n/2+1 else Sender is 1)
		//data[coodiY] represent Y1 cooridnate of Sender
		//data[Dist1] represent X2 cooridnate of Sender //here Dist1 temporarily stores the position of Sender
		//data[Dist2] represent Y2 cooridnate of Sender
		dist = passive_Measuring ((id-1) % (n/2));
		min11 = abs((data[coodiY] - y1)*(data[coodiY] - y1) + (data[coodiX]-x1)*(data[coodiX]-x1) - dist*dist);
		min12 = abs((data[coodiY] - y2)*(data[coodiY] - y2) + (data[coodiX]-x2)*(data[coodiX]-x2) - dist*dist);
		min21 = abs((data[Dist2] - y1)*(data[Dist2] - y1) + (data[Dist1]-x1)*(data[Dist1] - x1) - dist*dist);
		min22 = abs((data[Dist2] - y2)*(data[Dist2] - y2) + (data[Dist1]-x1)*(data[Dist1] - x1) - dist*dist);

		x=x1;
		y=y1;
		min = min11;

		if (min12 < min) {
			min = min12;
			x = x1;
			y = y1;
		}	
		if (min21 < min){
			min = min21;
			x = x2;
			y = y2;
		}
		if (min22 < min){
			min = min22;
			x = x2;
			y = y2;
		}
	}
}


