#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	MOTOR_SLEEP = 0,
	MOTOR_AWAKE = 1
}Motor_State;

typedef enum
{
	MOTOR_SLOW_DECAY_L = 0,
	MOTOR_SLOW_DECAY_H = 1,
	MOTOR_FAST_DECAY = 2
}Motor_Decay;

typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1,
	MOTOR_ALL = 2
}Motor_Selected;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//Motor Controller
#define nFAULT_LEFT								GPIO_Pin_5
#define nFAULT_LEFT_PORT					GPIOC
#define MODE1_LEFT								GPIO_Pin_4
#define MODE1_LEFT_PORT						GPIOC
#define MODE2_LEFT								GPIO_Pin_0
#define MODE2_LEFT_PORT						GPIOB
#define nSLEEP_LEFT								GPIO_Pin_6
#define nSLEEP_LEFT_PORT					GPIOA

#define Vpropi_LEFT								GPIO_Pin_4	 //ADC4
#define Vpropi_LEFT_PORT					GPIOA

#define nFAULT_RIGHT							GPIO_Pin_14
#define nFAULT_RIGHT_PORT					GPIOE
#define MODE1_RIGHT								GPIO_Pin_13
#define MODE1_RIGHT_PORT					GPIOE
#define MODE2_RIGHT								GPIO_Pin_15
#define MODE2_RIGHT_PORT					GPIOE
#define nSLEEP_RIGHT							GPIO_Pin_1
#define nSLEEP_RIGHT_PORT					GPIOB

#define Vpropi_RIGHT							GPIO_Pin_5 //ADC5
#define Vpropi_RIGHT_PORT					GPIOA

/* Exported functions ------------------------------------------------------- */
int Motor_State_Conf(Motor_State cond); //第一个需要调用的函数，控制8801工作或者休眠
int Motor_Decay_Conf(Motor_Selected selec, Motor_Decay decay); //Modify Motor Decat State
#endif
