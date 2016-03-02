#ifndef _MOVEMENT_CTR_H
#define _MOVEMENT_CTR_H

#include "stm32f10x.h"

void Degree_Exec(void);

void StopTurn(void);
void GoTurn(void);
void BackTurn(void);
void Go(void);
void Stop(void);

extern volatile uint8_t Type;
extern volatile int16_t StageGyro;
extern volatile uint8_t TurnFinish;
void SetDegree(int16_t DS);
void SetSpeed(int16_t S);
void SetType(uint8_t T);
void SetSource(uint8_t T);

#define GT 1
#define ST 2
#define BT 3
#define NT 0

#define Gyro_MPU 1
#define Gyro_Stage 2

int16_t DeltaDegree(void);

void SetState(int16_t degree, int16_t speed, uint8_t type);
void SetWay(int16_t degree, int16_t speed);

#endif
