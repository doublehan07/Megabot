/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "JY901.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern double InputSpeed_L;
extern double InputSpeed_R;
extern u8 PWMInited;
extern int32_t distance[2];
extern uint8_t state[2];

extern uint32_t count[2];
extern uint32_t count_t[2];

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
void ADC_PWM_Motor_Init(void);
void ADC_PWM_Motor_Exec(void); //在SysTick中执行，1~10ms一次
void Sampling_Tick_Speed(void);

#endif /* __MOTOR */
