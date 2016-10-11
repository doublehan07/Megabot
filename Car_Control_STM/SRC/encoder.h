#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"
#include "motor_control.h"
#include "imu.h"

#ifdef CAR_0
#define RATIO_R_L 1.017
#define TICK2DIS 1570.0 / 10000
#define ANGLE_OFFSET 0
#endif

#ifdef CAR_1
#define RATIO_R_L 1.02
#define TICK2DIS 1590.0 / 10000
#define ANGLE_OFFSET 1
#endif

#ifdef CAR_2
#define RATIO_R_L 0.993
#define TICK2DIS 1570.0 / 10000
#define ANGLE_OFFSET 1
#endif

#ifdef CAR_3
#define RATIO_R_L 1.023
#define TICK2DIS 1580.0 / 10000
#define ANGLE_OFFSET 1
#endif

void Save_Speed(void);
void Set_Speed_CCR(u8 corr);
void Angle_Corr(void);
void Set_CCR_PID(u16 ccr);
#ifdef CALIBRATE_ENCODER
void Set_CCR_PID_2(u16 ccr, u16 cycle);
#endif
void Set_Speed_PID(u8 speed);
void Set_PID_Stop(void);
float Getdx(void);
float Getdy(void);
void Read_Displacement(float *x, float *y);

#endif
