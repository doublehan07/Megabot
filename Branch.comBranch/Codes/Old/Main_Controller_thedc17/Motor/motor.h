/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "JY901.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	MOTOR_LEFT = 0,
	MOTOR_RIGHT = 1,
}Motor_Selected;


/* Exported constants --------------------------------------------------------*/
extern u8 MOTOR_RUNNING_FLAG;
extern u8 MOTOR_TURNNING_FLAG;

/* Exported macro ------------------------------------------------------------*/
//DRV8801
#define PHASE_LEFT													GPIO_Pin_10
#define ENABLE_LEFT													GPIO_Pin_0 //TIM1_CH2N
#define ENABLE_LEFT_Source									GPIO_PinSource0
#define ENABLE_LEFT_PWMO										CCR2

#define PHASE_RIGHT													GPIO_Pin_11
#define ENABLE_RIGHT												GPIO_Pin_1 //TIM1_CH3N
#define ENABLE_RIGHT_Source									GPIO_PinSource1
#define ENABLE_RIGHT_PWMO										CCR3

#define ENABLE_TIM													GPIO_AF_TIM1
#define PWM_TIM															TIM1
#define MOTOR_GPIO													GPIOB

//Encoder
#define ENCODER_LEFT_A											GPIO_Pin_0
#define ENCODER_LEFT_A_Source								EXTI_PinSource0
#define ENCODER_LEFT_A_Line									EXTI_Line0
#define ENCODER_LEFT_A_IRQn									EXTI0_IRQn

#define ENCODER_LEFT_B											GPIO_Pin_1

#define ENCODER_RIGHT_A											GPIO_Pin_2
#define ENCODER_RIGHT_A_Source							EXTI_PinSource2
#define ENCODER_RIGHT_A_Line								EXTI_Line2
#define ENCODER_RIGHT_A_IRQn								EXTI2_IRQn

#define ENCODER_RIGHT_B											GPIO_Pin_3

#define ENCODER_GPIO												GPIOA
#define ENCODER_PORT_SOURCE									EXTI_PortSourceGPIOA

/* Exported functions ------------------------------------------------------- */
void Motor_Init(void);
void Sampling_Tick_Speed(void);
void Motor_Setspeed(int16_t speedL, int16_t speedR);
void Motor_Set(u16 val);
void Motor_SetAngle(int16_t angle, u8 isRelative);

#endif /* __MOTOR */
