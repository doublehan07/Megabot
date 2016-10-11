#include "motor_control.h"

static u16 CCR_LEFT = 0;
static u16 CCR_RIGHT = 0;
static int8_t INS_Param = 0;

// 0--left, 1--right, 2--both
void Motor_Set_Direc(u8 left_or_right, u8 if_forward)
{
	if(left_or_right == 0)
	{
		if(if_forward)
		{
			GPIO_SetBits(PHASE_GPIO, PHASE_PIN_LEFT);
			GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_LEFT_2);
		}
		else
		{
			GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_LEFT);
			GPIO_SetBits(PHASE_GPIO, PHASE_PIN_LEFT_2);
		}
	}
	else if(left_or_right == 1)
	{
		if(if_forward)
		{
			GPIO_SetBits(PHASE_GPIO, PHASE_PIN_RIGHT);
			GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_RIGHT_2);
		}
		else
		{
			GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_RIGHT);
			GPIO_SetBits(PHASE_GPIO, PHASE_PIN_RIGHT_2);
		}
	}
	else
	{
		if(if_forward)
			INS_Param = 1;
		else
			INS_Param = -1;
		Motor_Set_Direc(0, if_forward);
		Motor_Set_Direc(1, if_forward);
	}
}

void Motor_Turn(u8 left_or_right)
{
	INS_Param = 0;
	Motor_Set_Direc(0, left_or_right);
	Motor_Set_Direc(1, !left_or_right);
}

void Motor_Stop(u8 left_or_right)
{
	if(left_or_right == 0)
	{
		GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_LEFT);
		GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_LEFT_2);
	}
	else if(left_or_right == 1)
	{
		GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_RIGHT);
		GPIO_ResetBits(PHASE_GPIO, PHASE_PIN_RIGHT_2);
	}
	else
	{
		Motor_Stop(0);
		Motor_Stop(1);
	}
}

void Motor_Brake(u8 left_or_right)
{
	if(left_or_right == 0)
	{
		GPIO_SetBits(PHASE_GPIO, PHASE_PIN_LEFT);
		GPIO_SetBits(PHASE_GPIO, PHASE_PIN_LEFT_2);
	}
	else if(left_or_right == 1)
	{
		GPIO_SetBits(PHASE_GPIO, PHASE_PIN_RIGHT);
		GPIO_SetBits(PHASE_GPIO, PHASE_PIN_RIGHT_2);
	}
	else
	{
		Motor_Brake(0);
		Motor_Brake(1);
	}
}

void Motor_Set_CCR(u8 left_or_right, short CCR)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	CCR = (CCR < 0)? 0 : CCR;
	CCR = (CCR > PWM_PERIOD) ? PWM_PERIOD : CCR;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	if(left_or_right == 0)
	{
		CCR_LEFT = CCR;
		TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	}
	else if(left_or_right == 1)
	{
		CCR_RIGHT = CCR;
		TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	}
	else
	{
		CCR_LEFT = CCR;
		CCR_RIGHT = CCR;
		TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
		TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	}
}

void Motor_Set_Speed(u8 left_or_right, u8 speed)
{
	u16 CCR;
	
	speed = (speed > 100)? 100:speed;
	CCR = PWM_PERIOD * speed / 100;
	
	Motor_Set_CCR(left_or_right, CCR);
}

u16 Get_CCR(u8 left_or_right)
{
	if(left_or_right == 0)
		return CCR_LEFT;
	else
		return CCR_RIGHT;
}

int8_t Get_INS_Param(void)
{
	return INS_Param;
}
