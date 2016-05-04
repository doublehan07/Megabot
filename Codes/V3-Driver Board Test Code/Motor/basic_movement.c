/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

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
		GPIO_ResetBits(CONTROL_GPIO, nSLEEP);
	}
	else //Motor awake
	{
		GPIO_SetBits(CONTROL_GPIO, nSLEEP);
	}
}

void Motor_If_FastDecay(u8 if_fast_decay)
{
	if(if_fast_decay == 0) //slow decay low voltage, mode1 = 1, mode2 = 0
	{
		GPIO_SetBits(CONTROL_GPIO, MODE1);
		GPIO_ResetBits(CONTROL_GPIO, MODE2);
		//slow decay high voltage, mode1 = 1, mode2 = 1
	}
	else //fast decay, mode1 = 0
	{
		GPIO_ResetBits(CONTROL_GPIO, MODE1);
	}
}

void Motor_If_Forward(u8 if_forward)
{
	if(if_forward) //forward
	{
		GPIO_SetBits(SIGNAL_GPIO, PHASE_PIN);
	}
	else //backward
	{
		GPIO_ResetBits(SIGNAL_GPIO, PHASE_PIN);
	}
}

void Motor_Set_Speed(u16 speed)
{
	speed = speed > 1000 ? 1000 : speed;
	PWM_TIM -> ENABLE_PWMO = speed;
}
