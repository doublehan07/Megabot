/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "strategy.h"

#include <math.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
Goto_Polar_Axis(r,theta)
|_
    Update_My_Axis(leader flag), get 2 distance and leader's sangle, return if_success
    |_
      Transfer_Dist(void), calculate angle in polar system, get my angle(r, theta), return if_success

    Calculate_Motion(void), calculate turning angle and distance, then get speed and approximate time

    use motor_function, read systick to get time, (before finish!)

    PID_Position_Adjustion(r, theta), use PID to get the proper positon, return if_success

return if_success
*/

/*
void SetTarget(double r, double theta, double Rect_x, double Rect_y, MyInfo myInfo)
{
	double dis = sqrt(r*r
		+myInfo.Polar_Axis[0]*myInfo.Polar_Axis[0]
		-2*r*myInfo.Polar_Axis[0]*cos(theta-myInfo.Polar_Axis[1]));//余弦定理
	double TarAngle = atan2(Rect_x-myInfo.Rect_Axis[0],Rect_y-myInfo.Rect_Axis[1]);//以x轴为极轴
	DisAndTurn(180/3.14159,0,dis/0.07);
}

void SetTarget_Polar_Axis(double r, double theta, MyInfo myInfo)
{
	double x = r * cos(theta), y = r * sin(theta);
	SetTarget(r, theta, x, y, myInfo);
}

void SetTarget_Rect_Axis(double x, double y, MyInfo myInfo)
{
	double r = sqrt(x*x+y*y), theta = atan2(x,y);
	SetTarget(r, theta, x, y, myInfo);
}
*/
