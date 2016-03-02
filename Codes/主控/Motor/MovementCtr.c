#include "MovementCtr.h"

#include "adc.h"
#include "pwm.h"
#include "motor_ctr.h"
#include "main.h"

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
