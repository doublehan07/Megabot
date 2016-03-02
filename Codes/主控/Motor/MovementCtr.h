#ifndef _MOVEMENT_CTR_H
#define _MOVEMENT_CTR_H

#include "stm32f4xx.h"

void ADC_PWM_Motor_Init(void);
void ADC_PWM_Motor_Exec(void); //在SysTick中执行，1~10ms一次
void Motor_Test(void);
void GoAndTurn(short angle, uint8_t isRelative, int16_t speed_l, int16_t speed_r);//angle为角度制

extern uint8_t isTuring;

/* Ports */
#define ADC_PORT GPIOC
#define ADC_PIN1 GPIO_Pin_4
#define ADC_PIN2 GPIO_Pin_5
#define ADC_Channel1 ADC_Channel_14
#define ADC_Channel2 ADC_Channel_15

#endif
