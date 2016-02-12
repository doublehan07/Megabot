#include "strategy.h"

struct RECEIVE ReceiveAI;
struct SEND SendAI;

uint8_t isFinished = 0 , toBeReceive = 0 , isParsed = 0 , requestToBeSent = 0;
char ReceivedPacket[23] , toBeParsedPack[23];
char receive_data[32];
char OurLocation[2] , lastOurLocation[2];

extern struct imu_data IMU_Data;

char getRivalID(void){
	if(ReceiveAI.ID==0x01)
		return 0x00;
	else if(ReceiveAI.ID==0x00)
		return 0x01;
}

void getOurLocation(char* OurLocation){
	lastOurLocation[0]= OurLocation[0];
	lastOurLocation[1]= OurLocation[1];
	switch(ReceiveAI.ID){
		case 0:
			OurLocation[0] = ReceiveAI.PeopleA[0];
			OurLocation[1] = ReceiveAI.PeopleA[1];
			break;
		case 1:
			OurLocation[0] = ReceiveAI.PeopleB[0];
			OurLocation[1] = ReceiveAI.PeopleB[1];
			break;
		default:
			break;
	}
}

void useDefendProp(void){
	int i = 3;
	if(ReceiveAI.PropI==0x01){
		for(;i>0;i--){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropI);}
	}else if(ReceiveAI.PropII==0x01){
		for(;i>0;i--){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropII);}		
	}else if(ReceiveAI.PropI==0x02){
		for(;i>0;i--){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropI);}
	}else if(ReceiveAI.PropII==0x02){
		for(;i>0;i--){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropII);}		
	}else{
		for(;i>0;i--){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropI);}
	}
}

void parseReceivedPack(char *ReceivePacket)
{
//	int i = 0;
	ReceiveAI = *(struct RECEIVE *)ReceivePacket;
		
//	receive_data[i++] = ReceiveAI.ID;
//	receive_data[i++] = ReceiveAI.Status;
//	receive_data[i++] = ReceiveAI.WarningA;
//	receive_data[i++] = ReceiveAI.WarningB;
//	receive_data[i++] = ReceiveAI.WarningC;
//	receive_data[i++] = ReceiveAI.WarningD;
//		
//	receive_data[i++] = ReceiveAI.If_track;
//	receive_data[i++] = ReceiveAI.Prop_num;
//	receive_data[i++] = ReceiveAI.Track_cond;
//		
//	receive_data[i++] = ReceiveAI.PropI;
//	receive_data[i++] = ReceiveAI.PropII;
//		
//	receive_data[i++] = ReceiveAI.PeopleA[0];
//	receive_data[i++] = ReceiveAI.PeopleA[1];
//	receive_data[i++] = ReceiveAI.PeopleB[0];
//	receive_data[i++] = ReceiveAI.PeopleB[1];
//	receive_data[i++] = ReceiveAI.PeopleC[0];
//	receive_data[i++] = ReceiveAI.PeopleC[1];
//	receive_data[i++] = ReceiveAI.PeopleD[0];
//	receive_data[i++] = ReceiveAI.PeopleD[1];
//	receive_data[i++] = ReceiveAI.PropA[0];
//	receive_data[i++] = ReceiveAI.PropA[1];
//	receive_data[i++] = ReceiveAI.PropB[0];
//	receive_data[i++] = ReceiveAI.PropB[1];
//	receive_data[i++] = ReceiveAI.PropC[0];
//	receive_data[i++] = ReceiveAI.PropC[1];
//	receive_data[i++] = ReceiveAI.PropD[0];	
//	receive_data[i++] = ReceiveAI.PropD[1];	
//		
//	receive_data[i++] = ReceiveAI.Source;	
//	receive_data[i++] = ReceiveAI.Target;	
//	receive_data[i++] = ReceiveAI.PropNow;	
//	receive_data[i++] = ReceiveAI.CheckD;	
//	receive_data[i++] = ReceiveAI.CheckA;	
	//printf("h");
	isParsed = 1 ;
}

/**
装包函数
装包的时候看一眼，我们使用的道具是道具一还是道具二
如果是道具二，没问题走你
如果是道具一，还要把ReceiveAI里面的道具二变成道具一，以免后面的解析函数用错道具
**/

void parseSendPack(char OurID, char TargetID, char Prop)
{
//	if(Prop==ReceiveAI.PropI){
//		ReceiveAI.PropI = ReceiveAI.PropII;
//		ReceiveAI.PropII = 0;
//	}else if(Prop==ReceiveAI.PropII){
//		ReceiveAI.PropII = 0;
//	}
	SendAI.OurID = OurID;
	SendAI.TargetID = TargetID;
	SendAI.PropID = Prop;
	SendAI.checkD = 0x0D;
	SendAI.checkA = 0x0A;
	//requestToBeSent = 1;
}

char getNearestPropID(void){
	return 0;
}
	
void useProp(char propID){
	
}

void sendDefaultPack(void){
		int i = 0 ;
		for(;i<3;i++){
			printf("%c",*((uint8_t *)&SendAI+i));
		}
}

void sendPack(char OurID, char TargetID, char Prop){
	parseSendPack(OurID,TargetID,Prop);
	sendDefaultPack();
}


void checkWhetherToBeAttacked(void){
	if(ReceiveAI.Target&(1<<ReceiveAI.ID) && (ReceiveAI.PropNow&0xF0)!=0){
		switch(checkWhetherPossessDefendProp()){
			case 1:
				sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropI);break;
			case 2:
				sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropII);break;
			default:
				break;
		}
	}
}

int checkWhetherPossessDefendProp(void){
	if(ReceiveAI.PropI<=0x04){
		return 1;
	}else if(ReceiveAI.PropII<=0x04){
		return 2;
	}else
		return 0;
}


int checkWhetherPossessAttackProp(void){
	if((ReceiveAI.PropI>0x04)&&(ReceiveAI.PropI<0x09)){
		return 1;
	}else if((ReceiveAI.PropII>0x04)&&(ReceiveAI.PropII<0x09)){
		return 2;
	}else
		return 0;
}

void useAttackProp(void){
		switch(checkWhetherPossessAttackProp()){
			case 1:
				sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropI);break;
			case 2:
				sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropII);break;
			default:
				break;
		}
}

//void checkWhetherToPickProp(){
//	int i = ReceiveAI.Prop_num , MinDistanceSquare = 400 , DistanceSquare , toPersue;
//	for(;i>0;i--){
////		if((IMU_Data.yaw/10-prop[i]<90)||(IMU_Data.yaw/10-prop[i]>-90)){
////			DistanceSquare = ((int)OurLocation[0]-(int)ReceiveAI.PropA[0])*((int)OurLocation[0]-(int)ReceiveAI.PropA[0]) + ((int)OurLocation[1]-(int)ReceiveAI.PropA[1])*((int)OurLocation[1]-(int)ReceiveAI.PropA[1]);
////			if(DistanceSquare < MinDistanceSquare){
////				MinDistanceSquare = DistanceSquare;
////				toPersue = i;
////			}
////		}
//	}
//	checkWhetherPassedNearbyProp(toPersue);
//}

///**
//查看道具附近是否有小车经过的点
//有的话headTo该道具
//**/
//void checkWhetherPassedNearbyProp(int toPersue){
//	switch(toPersue){
//		case 1: checkWhetherPassedNearbyPosition(ReceiveAI.PropA[0],ReceiveAI.PropA[1]);break;
//		case 2: checkWhetherPassedNearbyPosition(ReceiveAI.PropB[0],ReceiveAI.PropB[1]);break;
//		case 3: checkWhetherPassedNearbyPosition(ReceiveAI.PropC[0],ReceiveAI.PropC[1]);break;
//		case 4: checkWhetherPassedNearbyPosition(ReceiveAI.PropD[0],ReceiveAI.PropD[1]);break;
//		default: break;
//	}
//}

//void checkWhetherPassedNearbyPosition(char x,char y){
//	
//}

void useTrapProp(void){
	switch(checkWhetherPossessTrapProp()){
			case 1:
				sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropI);break;
			case 2:
				sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropII);break;
			default:
				break;
		}
}

int checkWhetherPossessTrapProp(void){
	if(ReceiveAI.PropI>0x08){
		return 1;
	}else if(ReceiveAI.PropII>0x08){
		return 2;
	}else
		return 0;
}

void getProp(void){
	sendPack(ReceiveAI.ID, ReceiveAI.ID, 0x0E);
	//sendDefaultPack();
}


char goingIN(void){
	char i = 0;
	int CurrentDistanceSquare , lastCurrentDistanceSquare;
	
	CurrentDistanceSquare = ((int)OurLocation[0]-(int)0x80)*((int)OurLocation[0]-(int)0x80) + ((int)OurLocation[1]-(int)0x80)*((int)OurLocation[1]-(int)0x80);
	lastCurrentDistanceSquare = ((int)lastOurLocation[0]-(int)0x80)*((int)lastOurLocation[0]-(int)0x80) + ((int)lastOurLocation[1]-(int)0x80)*((int)lastOurLocation[1]-(int)0x80);
	
	return (CurrentDistanceSquare<lastCurrentDistanceSquare)?1:0;
}
