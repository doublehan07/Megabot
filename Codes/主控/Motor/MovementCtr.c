#include "MovementCtr.h"

#include "adc.h"
#include "pwm.h"
#include "motor_ctr.h"
#include "port.h"

void ADC_PWM_Motor_Init()
{
	ADC_Init_User();
	PWM_Init();
	MOTOR_Init();
}

void ADC_PWM_Motor_Exec()
{
	Motor_Exec();
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
