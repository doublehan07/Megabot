#ifndef _PWM_H
#define _PWM_H

#include "stm32f4xx.h"

void PWM_Init(void);

#define PWM_Motor_1  (TIM3->CCR1)
#define PWM_Motor_2  (TIM3->CCR2)

#endif

