#include "MovementCtr.h"

#include "adc.h"
#include "pwm.h"
#include "motor_ctr.h"
#include "port.h"
#include "JY901.h"

short Angle_now, Angle_set;//模块正放则逆时针增大
int16_t Speed_set[2];
uint8_t isTuring;

void Movement_Exec(void);

void ADC_PWM_Motor_Init()
{
	ADC_Init_User();
	PWM_Init();
	MOTOR_Init();
}

void ADC_PWM_Motor_Exec()
{
	Motor_Exec();
	Movement_Exec();
}

void Motor_Test()
{
	MotorSpeedSet[0] = 60;
	MotorSpeedSet[1] = 60;
	Delay(3000);
	MotorSpeedSet[0] = 30;
	MotorSpeedSet[1] = 30;
	Delay(3000);
	MotorSpeedSet[0] = -30;
	MotorSpeedSet[1] = -30;
	Delay(3000);
	MotorSpeedSet[0] = -60;
	MotorSpeedSet[1] = -60;
	Delay(3000);
	MotorSpeedSet[0] = 0;
	MotorSpeedSet[1] = 0;
	Delay(3000);
}

void GoAndTurn(short angle, uint8_t isRelative, int16_t speed_l, int16_t speed_r)
{
	Angle_set = isRelative ? Angle_now+angle : angle;
	Angle_set = (Angle_set + 720) % 360;
	Speed_set[0] = speed_l;
	Speed_set[1] = speed_r;
	if (Speed_set[0] > 100)
		Speed_set[0] = 100;
	if (Speed_set[0] < -100)
		Speed_set[0] = -100;
	if (Speed_set[1] > 100)
		Speed_set[1] = 100;
	if (Speed_set[1] < -100)
		Speed_set[1] = -100;
}


#define P 1
#define D 0

void Movement_Exec()
{
	short dA,L,R,Angle_last = Angle_now;
	Angle_now = (int32_t)stcAngle.Angle[2] * 180.0 / 32768;
	dA = (Angle_set - Angle_now + 360) % 360;
	if(dA < 5 && dA > -5)
	{	
		L=R=0;
	}
	else if (dA < 180) //turn left
	{
		R = dA * P - D * ((Angle_now - Angle_last + 360) % 360);
		L = -R;
	}
	else //turn right
	{
		L = (360-dA) * P - D * ((Angle_last - Angle_now + 360) % 360);
		R = -L;
	}
	MotorSpeedSet[0] = Speed_set[0]+L;
	MotorSpeedSet[1] = Speed_set[1]+R;
	if (MotorSpeedSet[0] > 100)
		MotorSpeedSet[0] = 100;
	if (MotorSpeedSet[0] < -100)
		MotorSpeedSet[0] = -100;
	if (MotorSpeedSet[1] > 100)
		MotorSpeedSet[1] = 100;
	if (MotorSpeedSet[1] < -100)
		MotorSpeedSet[1] = -100;
}
