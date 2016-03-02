#ifndef _MOVEMENT_CTR_H
#define _MOVEMENT_CTR_H

#include "stm32f4xx.h"

void ADC_PWM_Motor_Init(void);
void ADC_PWM_Motor_Exec(void); //在SysTick中执行，1~10ms一次

#endif
