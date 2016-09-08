/**
  ******************************************************************************
  * 电机控制代码
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int Motor_State_Conf(Motor_State cond) //第一个需要调用的函数，控制8801工作或者休眠
{
	switch(cond)
	{
		//nSLEEP = 0
		case MOTOR_SLEEP: \
											GPIO_ResetBits(nSLEEP_LEFT_PORT, nSLEEP_LEFT); \
											GPIO_ResetBits(nSLEEP_RIGHT_PORT, nSLEEP_RIGHT); \
											break;
		//nsleep = 1
		case MOTOR_AWAKE: \
											GPIO_SetBits(nSLEEP_LEFT_PORT, nSLEEP_LEFT); \
											GPIO_SetBits(nSLEEP_RIGHT_PORT, nSLEEP_RIGHT); \
											break;
		//default: return -1;	break;
	}
	return 0;
}

int Motor_Decay_Conf(Motor_Selected selec, Motor_Decay decay) //Modify Motor Decat State
{
	if(decay == MOTOR_SLOW_DECAY_L) //MODE1 = 1, MODE2 = 0
	{
		switch(selec)
		{
			case MOTOR_LEFT:	\
												GPIO_SetBits(MODE1_LEFT_PORT, MODE1_LEFT); \
												GPIO_ResetBits(MODE2_LEFT_PORT, MODE2_LEFT); \
												break;
			case MOTOR_RIGHT:	\
												GPIO_SetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); \
												GPIO_ResetBits(MODE2_RIGHT_PORT, MODE2_RIGHT); \
												break;
			case MOTOR_ALL:	\
												GPIO_SetBits(MODE1_LEFT_PORT, MODE1_LEFT); \
												GPIO_SetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); \
												GPIO_ResetBits(MODE2_LEFT_PORT, MODE2_LEFT); \
												GPIO_ResetBits(MODE2_RIGHT_PORT, MODE2_RIGHT); \
												break;
			//default: return -1; break;
		}
	}
	else if(decay == MOTOR_SLOW_DECAY_H) //MODE1 = 1, MODE2 = 1
	{
		switch(selec)
		{
			case MOTOR_LEFT:	\
												GPIO_SetBits(MODE1_LEFT_PORT, MODE1_LEFT); \
												GPIO_SetBits(MODE2_LEFT_PORT, MODE2_LEFT); \
												break;
			case MOTOR_RIGHT:	\
												GPIO_SetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); \
												GPIO_SetBits(MODE2_RIGHT_PORT, MODE2_RIGHT); \
												break;
			case MOTOR_ALL:	\
												GPIO_SetBits(MODE1_LEFT_PORT, MODE1_LEFT); \
												GPIO_SetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); \
												GPIO_SetBits(MODE2_LEFT_PORT, MODE2_LEFT); \
												GPIO_SetBits(MODE2_RIGHT_PORT, MODE2_RIGHT); \
												break;
			//default: return -1; break;
		}
	}
	else //MODE1 = 0
	{
		switch(selec)
		{
			case MOTOR_LEFT: GPIO_ResetBits(MODE1_LEFT_PORT, MODE1_LEFT); break;
			case MOTOR_RIGHT: GPIO_ResetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); break;
			case MOTOR_ALL:	\
												GPIO_ResetBits(MODE1_LEFT_PORT, MODE1_LEFT); \
												GPIO_ResetBits(MODE1_RIGHT_PORT, MODE1_RIGHT); \
												break;
			//default: return -1; break;
		}	
	}
	return 0;
}
