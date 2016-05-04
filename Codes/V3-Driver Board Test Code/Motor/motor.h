/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//DRV8801
//Signal

#define PHASE_PIN								GPIO_Pin_2                                                                                                                       
#define ENABLE_PIN							GPIO_Pin_0								//TIM1_CH2N - PB0
#define ENABLE_Source 					GPIO_PinSource0
#define ENABLE_PWMO 						CCR2

#define ENABLE_TIM 							GPIO_AF_TIM1
#define PWM_TIM									TIM1
#define SIGNAL_GPIO							GPIOB

//Control - PCx
#define nFAULT									GPIO_Pin_0
#define MODE1										GPIO_Pin_1
#define MODE2										GPIO_Pin_2
#define nSLEEP									GPIO_Pin_3
#define CONTROL_GPIO						GPIOC

//ADC - PA1
#define Vpropi									GPIO_Pin_1
#define ADC_GPIO								GPIOA

/* Exported functions ------------------------------------------------------- */

void Motor_Init(void);

void Motor_If_Awake(u8 if_awake);
void Motor_If_FastDecay(u8 if_fast_decay);
void Motor_If_Forward(u8 if_forward);
void Motor_Set_Speed(u16 speed);



#endif /* __MOTOR */
