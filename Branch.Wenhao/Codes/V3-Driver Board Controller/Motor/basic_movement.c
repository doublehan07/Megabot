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
	speed = speed > 1000 ? 1000 : speed;
	
	if(left_or_right == 0) //select left motor
		PWM_TIM -> ENABLE_PWMO_LEFT = speed;
	
	else //select right motor
		PWM_TIM -> ENABLE_PWMO_RIGHT = speed;
}
