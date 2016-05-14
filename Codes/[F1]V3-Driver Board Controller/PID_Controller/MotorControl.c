#include "MotorControl.h"
#include "motor_pcb_interface.h"
#include "math.h"

/****************************************************************************************/
// global parameter to set the speed
// when we want to set a speed we could change this parameter
static int16_t currentSettedSpeed;

void setSpeed(int16_t speed)
{
	currentSettedSpeed = speed;
}
int16_t returnSpeed()
{
	return currentSettedSpeed;
}

/******************************************************************************************/

/*
* so easy!!
*/
uint16_t myABS(int16_t value)
{
	if(value >= 0)
		return value;
	else 
		return -value;
}

/*
* constrain the variation
*/
int16_t constrain(int16_t input, int16_t min, int16_t max)
{
	if(input > max)
		input = max;
	else if(input < max)
		input = min;
	return input;
}

/*
* convert the angle from -180~180 to 0~360
*/
int16_t angleTransform(int16_t angle_ABS180)
{
	if(angle_ABS180 > 0)
		return angle_ABS180 % 360;
	else
		return (MAX_ANGLE + angle_ABS180) % 360;
}

/*
* using PID to set the speed, we should keep the trail a straight line
* so we should read the current speed every 30ms and run the PID controller
* function every 30ms
* the correction of PID is the speed difference, so when we set the speed using API,
* we need to plus the correction with a const speed which we received 
*/
static int16_t LastError = 0;     // global error [-1]
static int16_t PrevError = 0;     // global error [-2]
void MotorSpeedPID(int16_t speed)
{
	// get current speed from conuter
	uint16_t leftSpeed = Get_Speed(LEFT);
	uint16_t rightSpeed = Get_Speed(RIGHT);

	double Proportion = P_DATA_SPEED;  // Proportional Const
    double Integral   = I_DATA_SPEED;  // Integral Const
    double Derivative = D_DATA_SPEED;  // Derivative Const

	// current error, we use left - right to represent the speed
	int16_t speedDiffernrce = leftSpeed - rightSpeed;

    // delta calculation     
    int16_t correctionSpeed = Proportion * speedDiffernrce  // E[k]
                            - Integral   * LastError                          // E[k-1]
                            + Derivative * PrevError;                       // E[k-2]
    PrevError = LastError; 
    LastError = speedDiffernrce;

    // implenment the PID settings
    Motor_Set_Speed(LEFT, speed - correctionSpeed);
    Motor_Set_Speed(RIGHT, speed + correctionSpeed);
}

/*
* use increment PID, before we reach the setted angle, we will keep calculating
* after seting angle, we will call another function to set the speed
*/
void Motor_Move(int16_t angle, u8 if_related, int16_t speed)
{
	double Proportion = P_DATA_ANGLE;  // Proportional Const
    double Integral   = I_DATA_ANGLE;  // Integral Const
    double Derivative = D_DATA_ANGLE;  // Derivative Const
	int16_t LastError = 0;             // Error [-1]
	int16_t PrevError = 0;             // Error [-2]
	
	int16_t angleNow = angleTransform(Inertia_Get_Angle_Yaw());
	int16_t angleSet = if_related ? angleNow + angleTransform(angle) : angleTransform(angle);
	speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

	// a loop to turn the angle setted
	// in this loop we will get the current angle and compare with the angle setted
	// using PID to achieve this goal

	// △u(k)=Kp[e(k)-e(k-1)]+Kie(k)+Kd[e(k)-2e(k-1)+e(k-2)]
	// △u(k)=Ae(k)-Be(k-1)+Ce(k-2)
	while(myABS(angleSet - angleNow) > ACCEPTTED_ERROR_ANGLE)
	{
		// current error and the correction
		int16_t currentError, correctionAngle;
    	currentError = angleSet - angleNow;

    	// delta calculation     
    	correctionAngle = Proportion * currentError  // E[k]              
                        - Integral * LastError     // E[k-1]
                        + Derivative * PrevError;  // E[k-2] 

    	PrevError = LastError;     // save last error, for next usage 
    	LastError = currentError;

		// implement
		// left
		if(angleSet - angleNow > 0 && angleSet - angleNow < 180) 
		{
			Motor_If_Forward(RIGHT, 1);
    		Motor_If_Forward(LEFT, 0);

			Motor_Set_Speed(LEFT, correctionAngle/MAX_SPEED); 
			Motor_Set_Speed(RIGHT, correctionAngle/MAX_SPEED);   
		}
		// right
		else if(angleSet - angleNow > 180) 
		{
			Motor_If_Forward(RIGHT, 0);
    		Motor_If_Forward(LEFT, 1);

			Motor_Set_Speed(LEFT, correctionAngle/MAX_SPEED); 
			Motor_Set_Speed(RIGHT, correctionAngle/MAX_SPEED);   
		}
		// left
		else if(angleSet - angleNow < 0 && angleSet - angleNow > -180) 
    	{
    		Motor_If_Forward(RIGHT, 1);
    		Motor_If_Forward(LEFT, 0);

			Motor_Set_Speed(LEFT, correctionAngle/MAX_SPEED); 
			Motor_Set_Speed(RIGHT, correctionAngle/MAX_SPEED);    
    	}
    	// right
    	else if(angleSet - angleNow < -180)    
    	{
    		Motor_If_Forward(RIGHT, 0);
    		Motor_If_Forward(LEFT, 1);

			Motor_Set_Speed(LEFT, correctionAngle/MAX_SPEED); 
			Motor_Set_Speed(RIGHT, correctionAngle/MAX_SPEED);   
    	}  
    	
		// update the current angle
		angleNow = angleTransform(Inertia_Get_Angle_Yaw());
	}
	
	// after the turning, we should set the speed for the straight line
	// but this function only set the parameter of the PID function
	// use a global parameter
	//currentSettedSpeed = speed;
}

/*
* we know the current position and we know the target position, so we need
* turn an approprite angle and go straight to reach the target position.
* When we go straight, we should know how far we have ran.
*/
void goToPosition(int16_t currentX, int16_t currentY, int16_t targetX, int16_t targetY)
{
	// get the distance
	int16_t straightDistance = sqrt(pow(targetY-currentY, 2) + pow(targetX-currentX, 2));
	
	// we should calculate the angle we should turn
	// int16_t currentAngle = Inertia_Get_Angle_Yaw();
	int16_t turnAngle = asin(myABS((targetY-currentY)/straightDistance));

	// turn the angle and set the speed 
	if(targetX > currentX && targetY > currentY)        // first quadrant
		Motor_Move(turnAngle, 1, MAX_SPEED);
	else if(targetX < currentX && targetY > currentY)   // second quadrant
		Motor_Move(-turnAngle, 1, MAX_SPEED);
	else if(targetX < currentX && targetY < currentY)   // third quadrant
		Motor_Move(turnAngle - MAX_ANGLE/2, 1, MAX_SPEED);
	else                                                // forth quadrant
		Motor_Move(MAX_ANGLE/2 - turnAngle, 1, MAX_SPEED);

	// run the distance
	// TODO
}
