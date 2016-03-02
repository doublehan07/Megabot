#ifndef _ADC_H
#define _ADC_H

#include "stm32f4xx.h"

void ADC_Init_User(void);

#define ADC_MOTOR1			0
#define ADC_MOTOR2			1
#define ADC_VOLREF			2

extern volatile uint16_t ADC_Voltage[3];

#endif

