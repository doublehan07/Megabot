#include "MovementCtr.h"

#include "adc.h"
#include "pwm.h"
#include "motor_ctr.h"
#include "port.h"
#include "JY901.h"

volatile short Angle_now, Angle_set;           //模块正放则逆时针增大
volatile int16_t Speed_set;
uint8_t MOTOR_TURNING_FLAG;

void Movement_Exec(void);                //  execute in SysTick,  1 to 10 ms, adjust the speed and angle

void ADC_PWM_Motor_Init()
{
	Delay(1000);
	ADC_Init_User();
	Angle_set = Angle_now = (short)(ANGLE_INPUT + 360) % 360;
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
	/*MotorSpeedSet[0] = 60;
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
	Delay(3000);*/
	
	GoAndTurn(30, 1, 0);
	while(isTuring);
	Delay(1000);
	GoAndTurn(-30, 1, 0);
	while(isTuring);
	Delay(1000);
	
	GoAndTurn(0, 0, 60);
	Delay(3000);
	
	GoAndTurn(0, 0, -60);
	Delay(3000);
	
	GoAndTurn(0, 0, 0);
	Delay(5000);
}

// set the angle and distance
void GoAndTurn(short angle, uint8_t isRelative, int16_t speed)
{
	Angle_set = isRelative ? Angle_now + angle : angle;
	Angle_set = (Angle_set + 720) % 360;
	Speed_set = speed;
	
	// max speed
	if (Speed_set > 100)
		Speed_set = 100;
	if (Speed_set < -100)
		Speed_set = -100;
}


#define RANGE 4

void Movement_Exec()
{
	short dA, L, R;
	//Angle_now = (int32_t)stcAngle.Angle[2] * 180 / 32768;
	Angle_now = (short)(ANGLE_INPUT + 360) % 360;
	dA = (Angle_set - Angle_now + 360) % 360;
	
	if(dA < RANGE && dA > -RANGE)
	{
		L=R=0;
		if (MOTOR_TURNING_FLAG)
			MOTOR_TURNING_FLAG--;
	}
	else if (dA < 180) //turn left
	{
		R = dA * P - D * (stcGyro.w[2] * 2000 / 32768) + 10;
		L = -R;
		MOTOR_TURNING_FLAG = 100;
	}
	else //turn right
	{
		L = (360-dA) * P + D * (stcGyro.w[2] * 2000 / 32768) + 10;
		R = -L;
		MOTOR_TURNING_FLAG = 100;
	}
	
	MotorSpeedSet[0] = Speed_set + L;
	MotorSpeedSet[1] = Speed_set + R;
	
	if (MotorSpeedSet[0] > 100)
		MotorSpeedSet[0] = 100;
	if (MotorSpeedSet[0] < -100)
		MotorSpeedSet[0] = -100;
	if (MotorSpeedSet[1] > 100)
		MotorSpeedSet[1] = 100;
	if (MotorSpeedSet[1] < -100)
		MotorSpeedSet[1] = -100;
}
