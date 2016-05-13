/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_pcb_interface.h"

/* Private typedef-----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Motor_If_Awake(u8 if_awake)
{
	if(if_awake == 0) //Motor sleep
	{
		GPIO_ResetBits(CONTROL_GPIO, nSLEEP_LEFT);
		GPIO_ResetBits(CONTROL_GPIO, nSLEEP_RIGHT);
	}
	else //Motor awake
	{
		GPIO_SetBits(CONTROL_GPIO, nSLEEP_LEFT);
		GPIO_SetBits(CONTROL_GPIO, nSLEEP_RIGHT);
	}
}

void Motor_If_FastDecay(u8 if_fast_decay)
{
	if(if_fast_decay == 0) //slow decay low voltage, mode1 = 1, mode2 = 0
	{
		GPIO_SetBits(CONTROL_GPIO, MODE1_LEFT);
		GPIO_SetBits(CONTROL_GPIO, MODE1_RIGHT);
		
		GPIO_ResetBits(CONTROL_GPIO, MODE2_LEFT);
		GPIO_ResetBits(CONTROL_GPIO, MODE2_RIGHT);
		
		//slow decay high voltage, mode1 = 1, mode2 = 1
	}
	else //fast decay, mode1 = 0
	{
		GPIO_ResetBits(CONTROL_GPIO, MODE1_LEFT);
		GPIO_ResetBits(CONTROL_GPIO, MODE1_RIGHT);
	}
}

void Motor_If_Forward(u8 left_or_right, u8 if_forward)
{
	if(left_or_right == 0) //select left motor
	{
		if(if_forward) //forward
		{
			GPIO_SetBits(SIGNAL_GPIO, PHASE_PIN_LEFT);
		}
		else //backward
		{
			GPIO_ResetBits(SIGNAL_GPIO, PHASE_PIN_LEFT);
		}
	}
	
	else //select right motor
	{
		if(if_forward) //forward
		{
			GPIO_SetBits(SIGNAL_GPIO, PHASE_PIN_RIGHT);
		}
		else //backward
		{
			GPIO_ResetBits(SIGNAL_GPIO, PHASE_PIN_RIGHT);
		}
	}
}

void Motor_Set_Speed(u8 left_or_right, u16 speed)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	speed = speed > 1000 ? 1000 : speed;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择模式PWM1，小于比较值时有效
		
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//TIM1CH1-TIM1CH4失能
	TIM_OCInitStructure.TIM_Pulse = speed;												//每次捕获的比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;					//有效电平为低电平（由于使用的是反相输出）
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;			//TIM1CH1-TIM1CH4输出状态
	
	if(left_or_right == 0) //select left motor
		TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	
	else //select right motor
		TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
}
