#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "port.h"
#include "imu.h"

void Motor_Set_Direc(u8 left_or_right, u8 if_forward);
void Motor_Turn(u8 left_or_right);
void Motor_Stop(u8 left_or_right);
void Motor_Brake(u8 left_or_right);
void Motor_Set_CCR(u8 left_or_right, short CCR);
void Motor_Set_Speed(u8 left_or_right, u8 speed);
u16 Get_CCR(u8 left_or_right);
int8_t Get_INS_Param(void);

#endif
