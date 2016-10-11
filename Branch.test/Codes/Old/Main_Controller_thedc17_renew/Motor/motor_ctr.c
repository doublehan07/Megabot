#include "motor_ctr.h"
#include "port.h"
#include "motor.h"

#define PWM_Motor_1  (TIM1->CCR2)
#define PWM_Motor_2  (TIM1->CCR3)

#define MT_DIR_PORT			GPIOB
#define MT1_DIR_PIN			GPIO_Pin_10
#define MT2_DIR_PIN			GPIO_Pin_11

#define MT1_DIR_N		GPIO_SetBits(MT_DIR_PORT, MT1_DIR_PIN)
#define MT1_DIR_P		GPIO_ResetBits(MT_DIR_PORT, MT1_DIR_PIN)
#define MT2_DIR_N		GPIO_SetBits(MT_DIR_PORT, MT2_DIR_PIN)
#define MT2_DIR_P		GPIO_ResetBits(MT_DIR_PORT, MT2_DIR_PIN)

int16_t MotorVoltage[2]={0};

void MOTOR_Voltage_Control(void)
{
	if(MotorVoltage[0]<0)
	{
		MT1_DIR_N;
		PWM_Motor_1=-MotorVoltage[0];
	}
	else
	{
		MT1_DIR_P;
		PWM_Motor_1=MotorVoltage[0];
	}
	
	
	if(MotorVoltage[1]<0)
	{
		MT2_DIR_N;
		PWM_Motor_2=-MotorVoltage[1];
	}
	else
	{
		MT2_DIR_P;
		PWM_Motor_2=MotorVoltage[1];
	}
}


volatile int16_t MotorSpeed[2]={0};
volatile int16_t MotorSpeedSet[2]={0};//Percentage

void MOTOR_CalculateMotorSpeed(void)
{
	MotorSpeed[0] = InputSpeed_L / 15 * 600;
	MotorSpeed[1] = InputSpeed_R / 15 * 600;
}

#define SCALE 1.1

void MOTOR_Speed_Control(void)
{
	uint8_t i;
	int32_t TargetSpeed;
	for(i=0;i!=2;i++)
	{
		int16_t LastVoltage = MotorVoltage[i]; //上一次的CCR
		if(MotorSpeedSet[i]>100)
			MotorSpeedSet[i]=100;
		else if(MotorSpeedSet[i]<-100)
			MotorSpeedSet[i]=-100;
		TargetSpeed = MotorSpeedSet[i] * 10;

		if ((TargetSpeed > 80 || TargetSpeed < -80) && (TargetSpeed < MotorSpeed[i] - 40 || TargetSpeed > MotorSpeed[i] + 40))
		{
			if (TargetSpeed && TargetSpeed * MotorSpeed[i] <= 0)//异号或停止
				MotorVoltage[i] = TargetSpeed > 0 ? 600 : -600;
			else
				MotorVoltage[i] = TargetSpeed * TargetSpeed * TargetSpeed * SCALE / MotorSpeed[i] / MotorSpeed[i];
		}
		else if (TargetSpeed < MotorSpeed[i] - 15 || TargetSpeed > MotorSpeed[i] + 15)
		{
			if (TargetSpeed && TargetSpeed * MotorSpeed[i] <= 0)//异号或停止
				MotorVoltage[i] = TargetSpeed * SCALE + 100;
			else
				MotorVoltage[i] = TargetSpeed * TargetSpeed * SCALE / MotorSpeed[i];
		}
		else
		{
			if (!TargetSpeed)
				MotorVoltage[i] = 0;
			if (TargetSpeed && TargetSpeed < MotorSpeed[i] - 3)
				MotorVoltage[i]++;
			if (TargetSpeed && TargetSpeed > MotorSpeed[i] + 3)
				MotorVoltage[i]--;
		}

		if (MotorVoltage[i] - LastVoltage > 100)
			MotorVoltage[i] = LastVoltage + 100;
		else if (MotorVoltage[i] - LastVoltage < -100)
			MotorVoltage[i] = LastVoltage - 100;
		if(MotorVoltage[i]>600)
			MotorVoltage[i]=600;
		else if(MotorVoltage[i]<-600)
			MotorVoltage[i]=-600;
	}
}


void Motor_Exec(void)
{
	if (PWMInited)
	{
		MOTOR_CalculateMotorSpeed();
		MOTOR_Speed_Control();
		MOTOR_Voltage_Control();
	}
}

