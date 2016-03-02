#ifndef _MOTOR_CTR_H
#define _MOTOR_CTR_H

#include "stm32f4xx.h"

void MOTOR_Init(void);
void Motor_Exec(void);
void MOTOR_Voltage_Control(void);

extern uint8_t PWMInited;
extern int16_t MotorVoltage[2];
extern volatile int16_t MotorSpeed[2];
extern volatile int16_t MotorSpeedSet[2];
extern volatile uint16_t MSZV[2];
//extern volatile uint16_t MSMR[2];

#endif

