/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
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
