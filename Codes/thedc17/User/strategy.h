#ifndef _STRATEGY_H
#define _STRATEGY_H
#include "stm32f4xx.h"
#include "usart.h"
#include "imu.h"
#include "math.h"

#define searchR 40
#define searchr 30
#define Pi 3.141

enum STATUS
{
	PREPARE,
	START,
	PAUSE,
	STOP,

	STATUS_NUM
};

enum PROP
{
	NONE = 0,

	//Defence
	SHELD = 1,
	IMMUNE = 2,
	CLEAR = 3,
	NC1 = 4, //NC

	//Attack
	BOMB = 5,
	DIZZY = 6,
	BOWLING = 7,
	MAGNET = 8,

	//Trap
	SWAMP = 9,
	LANDMINE = 10, //A
	NC2 = 11, //NC
	VORTEX = 12, //B

	//Special
	PICK = 13, //E

	PROP_NUM
};

struct RECEIVE //Because 32 received data from low bits, we need to reverse the ranks.
{	
	char WarningD : 1;
	char WarningC : 1;
	char WarningB : 1;
	char WarningA : 1;
	char Status : 2;
	char ID : 2;
	
	char Track_cond : 3;
	char Prop_num : 3;
	char If_track : 2; //enum Prop

	char PropII : 4;
	char PropI : 4;

	char PeopleA[2]; //{x,y}
	char PeopleB[2];
	char PeopleC[2];
	char PeopleD[2];
	char PropA[2];
	char PropB[2];
	char PropC[2];
	char PropD[2];

	char Target : 4; //May change
	char Source : 4; //May change

	char PropNow; //enum Prop
	char CheckD;
	char CheckA;
};

struct SEND
{
	char PropID : 4;
	char TargetID : 2;
	char OurID : 2;
	char checkD;
	char checkA;
};

extern struct RECEIVE ReceiveAI;
extern struct SEND SendAI;
void parseReceivedPack(char *ReceivePacket);
void parseSendPack(char OurID, char TargetID, char Prop);
void sendDefaultPack(void);
void sendPack(char OurID, char TargetID, char Prop);
void checkWhetherToBeAttacked(void);
int checkWhetherPossessDefendProp(void);
int checkWhetherPossessAttackProp(void);
void useAttackProp(void);
void checkWhetherToPickProp(void);
void checkWhetherPassedNearbyProp(int toPersue);
int checkWhetherPassedNearbyPosition(char x , char y);
void getProp(void);
void useAttackProp(void);
void useTrapProp(void);
void useDefendProp(void);
void checkWhetherInitIMUSuccess(void);
int16_t culculateAngleErr(int16_t myDirection ,int16_t PropDirection);
int16_t culculateAngle(char * destination);
void getPropLocation(char* propi , int i);
void chechWhetherStillShouldHeadTo(void);
char getRivalID(void);
int16_t formatAngle(int16_t angle);
void useDefendProp(void);
char goingIN(void);


#endif
