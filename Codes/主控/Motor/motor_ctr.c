#include "motor_ctr.h"

#include "pwm.h"
#include "adc.h"
#include "port.h"

#define MT_DIR_PORT			GPIOA
#define MT1_DIR_PIN			GPIO_Pin_4
#define MT2_DIR_PIN			GPIO_Pin_5

uint8_t PWMInited = 0;
uint16_t FullVoltage[2] = {1200, 1200};

void MOTOR_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t i;	
	
	//gpio initialize
	GPIO_InitStructure.GPIO_Pin = MT1_DIR_PIN|MT2_DIR_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(MT_DIR_PORT, &GPIO_InitStructure);

	Delay(10);
	
	//PreTest
	MOTOR_Voltage_Control();//Set PWM To 0
	Delay(10);
	
	for(i=0;i<2;i++)
	{
		if(ADC_Voltage[ADC_MOTOR1+i] > 1300 && ADC_Voltage[ADC_MOTOR1+i] < 1700)
			MSZV[i] = ADC_Voltage[ADC_MOTOR1+i];
	}

	Delay(10);
	
	for(i=0;i<2;i++)
	{
		if(ADC_Voltage[ADC_MOTOR1+i] > 1300 && ADC_Voltage[ADC_MOTOR1+i] < 1700)
			MSZV[i] = (MSZV[i] + ADC_Voltage[ADC_MOTOR1+i]) / 2;
	}
	PWMInited = 1;
}


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
		PWM_Motor_1=800+MotorVoltage[0];
	}
	else
	{
		MT1_DIR_P;
		PWM_Motor_1=MotorVoltage[0];
	}
	
	
	if(MotorVoltage[1]<0)
	{
		MT2_DIR_N;
		PWM_Motor_2=800+MotorVoltage[1];
	}
	else
	{
		MT2_DIR_P;
		PWM_Motor_2=MotorVoltage[1];
	}
}


volatile int16_t MotorSpeed[2]={0};
volatile int16_t MotorSpeedSet[2]={0};//Percentage

volatile uint16_t MSZV[2]={1487,1487};  //the adc motor voltage when motor is stop

void MOTOR_CalculateMotorSpeed(void)
{
	uint8_t i;
	int32_t Vi;
	
	for(i=0;i!=2;i++)
	{
		Vi = ((int32_t)ADC_Voltage[ADC_MOTOR1+i] - MSZV[i]) * 800;
		Vi /= ((Vi > 0) ? FullVoltage[0] : FullVoltage[1]);//Vi of 800
		if (Vi > 800)
			MotorSpeed[i] = 800;
		else if (Vi < -800)
			MotorSpeed[i] = -800;
		else
			MotorSpeed[i] = Vi;
	}
}


#define SCALE 1.1

void MOTOR_Speed_Control(void)
{
	uint8_t i;
	int32_t TargetSpeed;
	for(i=0;i!=2;i++)
	{
		int16_t LastVoltage = MotorVoltage[i];
		if(MotorSpeedSet[i]>100)
			MotorSpeedSet[i]=100;
		else if(MotorSpeedSet[i]<-100)
			MotorSpeedSet[i]=-100;
		TargetSpeed = MotorSpeedSet[i] * 8;

		if ((TargetSpeed > 80 || TargetSpeed < -80) && (TargetSpeed < MotorSpeed[i] - 40 || TargetSpeed > MotorSpeed[i] + 40))
		{
			if (TargetSpeed && TargetSpeed * MotorSpeed[i] <= 0)//ÒìºÅ»òÍ£Ö¹
				MotorVoltage[i] = TargetSpeed > 0 ? 800 : -800;
			else
				MotorVoltage[i] = TargetSpeed * TargetSpeed * TargetSpeed * SCALE / MotorSpeed[i] / MotorSpeed[i];
		}
		else if (TargetSpeed < MotorSpeed[i] - 15 || TargetSpeed > MotorSpeed[i] + 15)
		{
			if (TargetSpeed && TargetSpeed * MotorSpeed[i] <= 0)//ÒìºÅ»òÍ£Ö¹
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

		if (MotorVoltage[i] - LastVoltage > 400)
			MotorVoltage[i] = LastVoltage + 400;
		else if (MotorVoltage[i] - LastVoltage < -400)
			MotorVoltage[i] = LastVoltage - 400;
		if(MotorVoltage[i]>800)
			MotorVoltage[i]=800;
		else if(MotorVoltage[i]<-800)
			MotorVoltage[i]=-800;
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

