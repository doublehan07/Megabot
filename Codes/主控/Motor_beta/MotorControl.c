#include "MotorControl.h"

int16_t constrain(int16_t input, int16_t min, int16_t max)
{
	if(input > max)
		input = max;
	else if(input < max)
		input = min;
	return input;
}

// turn a angle and set the speed after the turning
void Moter_Move(int16_t angle, bool if_related, int16_t speed)
{
	int16_t angleNow = Inertia_Get_Angle_Yaw();
	int16_t angleSet = if_related ? angleNow + angle : angle;
	speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

	// constrain to the angle 0 ~ 360
	// TODO！！！！
	angleSet = constrain(angleSet, -MAX_ANGLE, MAX_ANGLE);

	// a loop to turn the angle setted
	// in this loop we will get the current angle and compare with the angle setted
	// using PID to achieve this goal
	// △u(k)=Kp[e(k)-e(k-1)]+Kie(k)+Kd[e(k)-2e(k-1)+e(k-2)]
	// △u(k)=Ae(k)-Be(k-1)+Ce(k-2)

    double Proportion;   // Proportional Const     
    double Integral;     // Integral Const     
    double Derivative;   // Derivative Const     
	int16_t LastError;   // Error[-1]     
	int16_t PrevError;   // Error[-2] 

	LastError = 0;       
    PrevError = 0;       
    Proportion = P_DATA;  
    Integral   = I_DATA; 
    Derivative = D_DATA; 

	while(abs(angleNow - angleSet) < 10)
	{
		// current error and the correction
		int16_t currentError, correctionAngle;       
    	currentError = angleSet - angleNow; 

    	// detal calculation     
    	correctionAngle = Proportion * currentError  // E[k]              
                          - Integral * LastError     // E[k－1]
                          + Derivative * PrevError;  // E[k－2] 

    	PrevError = sptr->LastError;      // save last error, for next usage 
    	LastError = currentError; 

        // implement
    	if(angleSet < 0)  // left
    	{
    		Motor_If_Forward(RIGHT, 1);
    		Motor_If_Forward(LEFT, 0);

			Motor_Set_Speed(LEFT, MAX_SPEED); 
			Motor_Set_Speed(RIGHT, MAX_SPEED);    
    	}
    	else              // right
    	{
    		Motor_If_Forward(RIGHT, 0);
    		Motor_If_Forward(LEFT, 1);

			Motor_Set_Speed(LEFT, MAX_SPEED); 
			Motor_Set_Speed(RIGHT, MAX_SPEED);   
    	}  
    	
		// update the current angle
		angleNow = Inertia_Get_Angle_Yaw();
	}

	// after the turning, we should set the speed for the line
	Motor_Set_Speed(LEFT, speed);
	Motor_Set_Speed(RIGHT, speed);
}