#include "encoder.h"

static u16 leftCounter = 0;
static u16 rightCounter = 0;
static u16 saveLeftCounter = 0;
static u16 saveRightCounter = 0;
static u16 Speed_CCR = 0;
static short Speed_CCR_INCREASE = 0;
static short ccr_correct = 0;
static float dx = 0;
static float dy = 0;

#ifdef CALIBRATE_ENCODER
static u16 l, r, c;
#endif

void EXTI9_5_IRQHandler(void) // Left B
{
	leftCounter++;
	EXTI_ClearITPendingBit(EXTI_Line7);
}

void EXTI2_IRQHandler(void) // Left A
{
	leftCounter++;
	EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI1_IRQHandler(void) // Right A
{
	rightCounter++;
	EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI0_IRQHandler(void) // Right B
{
	rightCounter++;
	EXTI_ClearITPendingBit(EXTI_Line0);
}

short Angle_Diff(short angle1, short angle2)
{
	short ret = angle1 - angle2;
	if(ret < -180)
		return ret + 360;
	if(ret > 180)
		return ret - 360;
	return ret;
}

void Save_Speed(void)
{
	float speed_err = 0;
	float dis;
	saveLeftCounter = leftCounter;
	saveRightCounter = rightCounter;
	#ifdef CALIBRATE_ENCODER
	l += leftCounter;
	r += rightCounter;
	if( (l + r) / 2 > c)
		Motor_Brake(2);
	#endif
	leftCounter = 0;
	rightCounter = 0;
	dis = (saveLeftCounter + saveRightCounter) * TICK2DIS / 2.0;
	dx += Get_INS_Param() * dis * cos(getAngleRad());
	dy += Get_INS_Param() * dis * sin(getAngleRad());
	if(Speed_CCR)
	{
		if(saveLeftCounter == 0 || saveRightCounter == 0)
		{
			Speed_CCR_INCREASE += 3;
		}
		else if(Speed_CCR_INCREASE != 0 && saveLeftCounter > 8 && saveRightCounter > 8)
		{
			Speed_CCR_INCREASE -= (saveLeftCounter + saveRightCounter - 16) / 10;
			Speed_CCR_INCREASE = (Speed_CCR_INCREASE < 0) ? 0:Speed_CCR_INCREASE;
		}
		speed_err = saveLeftCounter - saveRightCounter*RATIO_R_L;
		ccr_correct += speed_err * 5;
		Motor_Set_CCR(0, Speed_CCR + Speed_CCR_INCREASE - ccr_correct / 2);
		Motor_Set_CCR(1, Speed_CCR + Speed_CCR_INCREASE + ccr_correct / 2);
	}
}

void Set_CCR_PID(u16 ccr)
{
	if(Speed_CCR != ccr)
	{
		Speed_CCR_INCREASE = 0;
		Speed_CCR = ccr;
	}
}

#ifdef CALIBRATE_ENCODER
void Set_CCR_PID_2(u16 ccr, u16 cycle)
{
	c = cycle;
	if(Speed_CCR != ccr)
	{
		Speed_CCR_INCREASE = 0;
		Speed_CCR = ccr;
	}
}
#endif

void Set_Speed_PID(u8 speed)
{
	speed = (speed > 100)? 100:speed;
	Set_CCR_PID(PWM_PERIOD * speed / 100);
}

void Set_PID_Stop(void)
{
	Set_CCR_PID(0);
}

float Getdx(void)
{
	return dx;
}

float Getdy(void)
{
	return dy;
}

void Read_Displacement(float *x, float *y)
{
	float dis = (leftCounter + rightCounter) * TICK2DIS / 2.0;
	dx += Get_INS_Param() * dis * cos(getAngleRad());
	dy += Get_INS_Param() * dis * sin(getAngleRad());
	*x = dx;
	*y = dy;
	dx = 0;
	dy = 0;
	leftCounter = 0;
	rightCounter = 0;
}
