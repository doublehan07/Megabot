#ifndef __IMU_H
#define __IMU_H

#include "port.h"
#include "motor_control.h"
#include "encoder.h"
#include <string.h>
#include <math.h>

#define PI 3.1415926
#ifdef CAR_0
#define N 8
#else
#define N 4
#endif

struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};

struct SAcc
{
	short a[3];
	short T;
};

struct SGyro
{
	short w[3];
	short T;
};

struct SAngle
{
	short Angle[3];
	short T;
};

struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};

struct SQuat
{
	short Q[4];
};

struct SSatePre
{
	short SN;
	short PDOP;
	short HDOP;
	short VDOP;
};

float getAngleRad(void);
u16 getAngleDeg(void);
u8 SetAngleOffset(float angle);

#endif
