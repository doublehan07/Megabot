#ifndef _MOVEMENT_CTR_H
#define _MOVEMENT_CTR_H

#include "stm32f4xx.h"

/* Functions */
void ADC_PWM_Motor_Init(void);
void ADC_PWM_Motor_Exec(void); //在SysTick中执行，1~10ms一次
void GoAndTurn(short angle, uint8_t isRelative, int16_t speed);//angle为角度制
void DisAndTurn(short angle, uint8_t isRelative, int32_t dis);

/* Definitions */
#define MOTOR_TURNING_FLAG isTuring
#define MOTOR_RUNNING_FLAG isRunning
#define ANGLE_INPUT sAngle //尤其需要注意绝对角度，因为是与此输入相比较
#define P 1.2
#define D 0.1
extern uint8_t MOTOR_TURNING_FLAG;
extern uint8_t MOTOR_RUNNING_FLAG;

/* Ports */
#define ADC_PORT GPIOC
#define ADC_PIN1 GPIO_Pin_4
#define ADC_PIN2 GPIO_Pin_5
#define ADC_Channel1 ADC_Channel_14
#define ADC_Channel2 ADC_Channel_15

#endif
