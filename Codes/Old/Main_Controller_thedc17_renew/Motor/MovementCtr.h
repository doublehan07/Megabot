#ifndef _MOVEMENT_CTR_H
#define _MOVEMENT_CTR_H

#include "stm32f4xx.h"

/* Functions */
void GoAndTurn(short angle, uint8_t isRelative, int16_t speed);//angle为角度制
void DisAndTurn(short angle, uint8_t isRelative, int32_t dis);

/* Definitions */
#define MOTOR_TURNING_FLAG isTurning
#define MOTOR_RUNNING_FLAG isRunning
#define ANGLE_INPUT sAngle //尤其需要注意绝对角度，因为是与此输入相比较
#define P 0.3
#define D 0.1
#define MOTOR_SPEED 8 //1~8(default)
extern uint8_t MOTOR_TURNING_FLAG;
extern uint8_t MOTOR_RUNNING_FLAG;
#endif
