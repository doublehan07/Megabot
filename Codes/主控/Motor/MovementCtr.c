#include "MovementCtr.h"

#include "IIC_6050.h"
#include "motor_ctr.h"
#include "main.h"

volatile float Degree = 0;
volatile int16_t DegreeSet = 0;

volatile int16_t Speed = 0;
volatile uint8_t Type = 0;
volatile uint8_t Source = 2;
volatile int16_t StageGyro = 0;
volatile uint8_t TurnFinish = 1;

#define RANGE 4

void Degree_Exec(void)
{
	if (Source == Gyro_MPU)
	{
		if (Gyro > 0.5 || Gyro < -0.5)
			Degree += Gyro;
	}
	else if (Source == Gyro_Stage)
	{
			Degree += (float)(MotorSpeedSet[1] - MotorSpeedSet[0]) * 9 / 2;		
	}
	if (Degree > 240000)
		Degree -= 360000;
	if (Degree < -240000)
		Degree += 360000;
	switch (Type)
	{
		case GT: GoTurn(); break;
		case BT: BackTurn(); break;
		case ST: StopTurn(); break;
		default: Go();
	}
}

float P = 0.8;
//float D = 0.2;
int32_t MSS[2] = {0};

void StopTurn(void)
{
	if (Degree/1000 > DegreeSet + RANGE || Degree/1000 < DegreeSet - RANGE)
	{
		if (DegreeSet > 0 && DegreeSet - Degree/1000 > DegreeSet * 2 / 3)
		{
			MSS[0] = -100;
			MSS[1] = 100;
		}
		else if (DegreeSet < 0 && DegreeSet - Degree/1000 < DegreeSet * 2 / 3)
		{
			MSS[0] = 100;
			MSS[1] = -100;
		}
		else
		{
			MSS[0] = (Degree/1000 - DegreeSet) * P;
			MSS[1] = (DegreeSet - Degree/1000) * P;
			MSS[0] += MSS[0] > 0 ? 3 : -3;
			MSS[1] += MSS[1] > 0 ? 3 : -3;
		}
		if (MSS[0] > 100)
			MotorSpeedSet[0] = 100;
		else if (MSS[0] < -100)
			MotorSpeedSet[0] = -100;
		else
			MotorSpeedSet[0] = MSS[0];
		if (MSS[1] > 100)
			MotorSpeedSet[1] = 100;
		else if (MSS[1] < -100)
			MotorSpeedSet[1] = -100;
		else
			MotorSpeedSet[1] = MSS[1];
	}
	else
	{
		Go();
		if (!TurnFinish)
			MakeNextMove = 1;
		TurnFinish = 1;
		//MakeNextMove = (DegreeSet > 0 ? DegreeSet : -DegreeSet) / 40 + 1;
	}
}

void GoTurn(void)
{
	if (Degree/1000 > DegreeSet + RANGE || Degree/1000 < DegreeSet - RANGE)
	{
		MotorSpeedSet[0] = (Degree/1000 - DegreeSet) * P;
		MotorSpeedSet[1] = (DegreeSet - Degree/1000) * P;
//		if (MotorSpeedSet[0] <= 0) MotorSpeedSet[0] = 0;
//		else MotorSpeedSet[0] += 5;
//		if (MotorSpeedSet[1] <= 0) MotorSpeedSet[1] = 0;
//		else MotorSpeedSet[0] += 5;
		if(Speed <= 80)
		{
			MotorSpeedSet[0] += Speed;
			MotorSpeedSet[1] += Speed;
		}
		else
		{
			MotorSpeedSet[0] += 80;
			MotorSpeedSet[1] += 80;
		}			
		if (MotorSpeedSet[0] > 100)
			MotorSpeedSet[0] = 100;
		else if (MotorSpeedSet[0] < -100)
			MotorSpeedSet[0] = -100;
		if (MotorSpeedSet[1] > 100)
			MotorSpeedSet[1] = 100;
		else if (MotorSpeedSet[1] < -100)
			MotorSpeedSet[1] = -100;
	}
	else
	{
		Go();
		TurnFinish = 1;
//		ChangeFlag = 1;
	}
}

void BackTurn(void)
{
	if (Degree/1000 > DegreeSet + RANGE || Degree/1000 < DegreeSet - RANGE)
	{
		MotorSpeedSet[0] = (Degree/1000 - DegreeSet) * P;
		MotorSpeedSet[1] = (DegreeSet - Degree/1000) * P;
//		if (MotorSpeedSet[0] >= 0) MotorSpeedSet[0] = 0;
//		else MotorSpeedSet[0] -= 5;
//		if (MotorSpeedSet[1] >= 0) MotorSpeedSet[1] = 0;
//		else MotorSpeedSet[0] -= 5;
		if(Speed >= -80)
		{
			MotorSpeedSet[0] += Speed;
			MotorSpeedSet[1] += Speed;
		}
		else
		{
			MotorSpeedSet[0] -= 80;
			MotorSpeedSet[1] -= 80;
		}			
		if (MotorSpeedSet[0] > 100)
			MotorSpeedSet[0] = 100;
		else if (MotorSpeedSet[0] < -100)
			MotorSpeedSet[0] = -100;
		if (MotorSpeedSet[1] > 100)
			MotorSpeedSet[1] = 100;
		else if (MotorSpeedSet[1] < -100)
			MotorSpeedSet[1] = -100;
	}
	else
	{
		Go();
		TurnFinish = 1;
//		ChangeFlag = 1;
	}
}

void SetDegree(int16_t DS)
{
	Degree = 0;
	DegreeSet = DS;
	TurnFinish = 0;
}

void SetSpeed(int16_t S)
{
	if (Speed <= 0 && S > 0)
		Speed = S / 5;
	else if (Speed < S)
		Speed = (Speed + S) / 2;
	else
		Speed = S;
}

void Go(void)
{
	MotorSpeedSet[0] = MotorSpeedSet[1] = Speed;
}

void Stop(void)
{
	Speed = 0;
	Type = NT;
}

void SetType(uint8_t T)
{
	Type = T;
}

void SetState(int16_t degree, int16_t speed, uint8_t type)
{
	SetDegree(degree);
	SetSpeed(speed);
	SetType(type);
}

void SetWay(int16_t degree, int16_t speed)
{
	SetDegree(degree);
	SetSpeed(speed);
	if (speed > 0)
		SetType(GT);
	else if (speed < 0)
		SetType(BT);
	else
	{
		if (degree)
			SetType(ST);
		else
			SetType(NT);
	}
}

int16_t DeltaDegree(void)
{
	return (int16_t)(Degree/1000 - DegreeSet);
}

void SetSource(uint8_t T)
{
	Source = T;
}
