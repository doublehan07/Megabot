#include "MotorControl.h"

// constrain the variation
int16_t constrain(int16_t input, int16_t min, int16_t max)
{
	if(input > max)
		input = max;
	else if(input < max)
		input = min;
	return input;
}

// convert the angle from -180~180 to 0~360
int16_t angleTransform(int16_t angle_ABS180)
{
	if(angle_ABS180 > 0)
		return angle_ABS180 % 360;
	else
		return (MAX_ANGLE + angle_ABS180) % 360;
}

// turn a angle and set the speed after the turning
void Moter_Move(int16_t angle, bool if_related, int16_t speed)
{
	int16_t angleNow = angleTransform(Inertia_Get_Angle_Yaw());
	int16_t angleSet = if_related ? angleNow + angleTransform(angle) : angleTransform(angle);
	speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

	// a loop to turn the angle setted
	// in this loop we will get the current angle and compare with the angle setted
	// using PID to achieve this goal

	// △u(k)=Kp[e(k)-e(k-1)]+Kie(k)+Kd[e(k)-2e(k-1)+e(k-2)]
	// △u(k)=Ae(k)-Be(k-1)+Ce(k-2)

	double Proportion;   // Proportional Const     
    double Integral;     // Integral Const     
    double Derivative;   // Derivative Const     
	int16_t LastError;   // Error [-1]     
	int16_t PrevError;   // Error [-2] 

	LastError = 0;       
    PrevError = 0;       
    Proportion = P_DATA;  
    Integral   = I_DATA; 
    Derivative = D_DATA; 

	while(abs(angleSet - angleNow) > ACCEPTTED_ERROR)
	{
		// current error and the correction
		int16_t currentError, correctionAngle;       
    	currentError = angleSet - angleNow; 

    	// delta calculation     
    	correctionAngle = Proportion * currentError  // E[k]              
                          - Integral * LastError     // E[k-1]
                          + Derivative * PrevError;  // E[k-2] 

    	PrevError = LastError;  // save last error, for next usage 
    	LastError = currentError; 

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
	Motor_Set_Speed(LEFT, speed);
	Motor_Set_Speed(RIGHT, speed);
}