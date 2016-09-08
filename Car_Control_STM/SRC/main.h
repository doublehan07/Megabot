#ifndef __MAIN_H
#define __MAIN_H

#include "init.h"
#include "motor_control.h"
#include "bt_comm.h"
#include "encoder.h"
#include "wx_comm.h"
#include "imu.h"

#include <math.h>

#define ARRIVE_DIS_THRESHOLD 50
#define NEAR_ANGLE_THRESHOLD 10 * PI / 180
#define ANGLE_THRESHOLD_CHANGE_DIS 500
#define FAR_ANGLE_THRESHOLD 15 * PI / 180
#define TURNNING_ANGLE_THRESHOLD 5 * PI / 180
#ifdef CAR_1
#define TURNNING_SPEED_FAST 18
#define TURNNING_SPEED_SLOW 15
#endif
#ifdef CAR_2
#define TURNNING_SPEED_FAST 18
#define TURNNING_SPEED_SLOW 15
#else
#define TURNNING_SPEED_FAST 22
#define TURNNING_SPEED_SLOW 18
#endif

#endif
