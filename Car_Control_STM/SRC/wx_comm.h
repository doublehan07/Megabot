#ifndef __WX_COMM_H
#define __WX_COMM_H

#include "port.h"
#include "imu.h"
#include "encoder.h"

struct Route
{
	short x;
	short y;
	short target_x;
	short target_y;
	float angle_0;
};

struct Route GetRoute(void);
void SetShape(u16 *x, u16 *y);

#endif
