#include "MovementCtr.h"

#include "motor_ctr.h"
#include "motor.h"
#include "port.h"
#include "JY901.h"

#define MOTOR_DEBUG 1

volatile short Angle_now, Angle_set;//模块正放则逆时针增大
volatile int16_t Speed_set;
uint8_t MOTOR_TURNING_FLAG, MOTOR_RUNNING_FLAG;
int32_t distance[2] = {0,0};
int32_t dis_tar = 0;//*0.07cm
uint8_t state[2]={0,0};

#ifdef MOTOR_DEBUG
uint32_t count[2]={0,0};
uint32_t count_t[2]={0,0};
#endif

void Movement_Exec(void);

void ADC_PWM_Motor_Init()
{
	Delay(1000);
	Angle_set = Angle_now = (short)(ANGLE_INPUT + 360) % 360;
	Motor_Init();
}

void ADC_PWM_Motor_Exec()
{
	Motor_Exec();
	Movement_Exec();
}

void GoAndTurn(short angle, uint8_t isRelative, int16_t speed)
{
	Angle_set = isRelative ? Angle_now+angle : angle;
	Angle_set = (Angle_set + 720) % 360;
	Speed_set = speed;
	if (Speed_set > 100)
		Speed_set = 100;
	if (Speed_set < -100)
		Speed_set = -100;
}

void DisAndTurn(short angle, uint8_t isRelative, int32_t dis)
{
	Angle_set = isRelative ? Angle_now+angle : angle;
	Angle_set = (Angle_set + 720) % 360;
	Speed_set = dis > 0 ? dis : -dis;
	dis_tar = dis;
	if (Speed_set > 40)
		Speed_set = 40;
	distance[0] = distance[1] = 0;
#ifdef MOTOR_DEBUG
	count[0] = count[1] = 0;
#endif
	MOTOR_RUNNING_FLAG = 100;
}
#define RANGE 5

void Movement_Exec()
{
	int16_t dA,L,R,Speed_set_t = Speed_set;
	//Angle_now = (int32_t)stcAngle.Angle[2] * 180 / 32768;
	Angle_now = (short)(ANGLE_INPUT + 360) % 360;
	dA = (Angle_set - Angle_now + 360) % 360;
	if(dA < RANGE || dA > 360-RANGE)
	{
		L=R=0;
		if (MOTOR_TURNING_FLAG)
			MOTOR_TURNING_FLAG--;
	}
	else if (dA < 180) //turn left
	{
		R = dA * P - D * (stcGyro.w[2] * 2000 / 32768) + 10;
		R = R > 0 ? R : 0;
		L = -R;
		MOTOR_TURNING_FLAG = 100;
	}
	else //turn right
	{
		L = (360-dA) * P + D * (stcGyro.w[2] * 2000 / 32768) + 10;
		L = L > 0 ? L : 0;
		R = -L;
		MOTOR_TURNING_FLAG = 100;
	}
	if (dis_tar)
	{
		int32_t dis = dis_tar - (distance[0] + distance[1]);
		if (dis > 160 || dis < -160)
		{
			Speed_set_t = dis > 0 ? Speed_set : -Speed_set;
			MOTOR_RUNNING_FLAG = 100;
		}
		else if (dis > 16 || dis < -16)
		{
			Speed_set_t = dis > 0 ? 15 : -15;
			MOTOR_RUNNING_FLAG = 100;
		}
		else
		{
			Speed_set_t = 0;
			if (MOTOR_RUNNING_FLAG)
				MOTOR_RUNNING_FLAG--;
			else
			{
				dis_tar = 0;
				distance[0] = distance[1] = 0;
				Speed_set = 0;
			}
		}
	}
	MotorSpeedSet[0] = Speed_set_t+L;
	MotorSpeedSet[1] = Speed_set_t+R;
	if (MotorSpeedSet[0] > 100)
		MotorSpeedSet[0] = 100;
	if (MotorSpeedSet[0] < -100)
		MotorSpeedSet[0] = -100;
	if (MotorSpeedSet[1] > 100)
		MotorSpeedSet[1] = 100;
	if (MotorSpeedSet[1] < -100)
		MotorSpeedSet[1] = -100;
}
