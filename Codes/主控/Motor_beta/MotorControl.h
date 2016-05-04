/***************************************************************************************

*  Used functions:
*  Motor_If_Forward(left_or_right, if_forward)       decide the direction of the motor
*  Motor_Set_Speed(left_or_right, speed)             set the speed
*  Inertia_Get_Angle_Yaw()  get the angle from the gyroscope(double)
*  
*  Output function:
*  Moter_Move(uint16_t angle, bool if_related, uint16_t speed)
*  	parameters:
*      angle: the angle that want to move
*      if_related: 1 means relative angle to current angle
*                  0 means absolutely angle
*      speed: the speed of the motor after turning
*
*  The angle we get from the gyroscope is from -180 to 180, but in order to simplify the
*  adjustion, we convert the scope from 0 to 360, clockwise represents turning right

****************************************************************************************/
#ifndef MOTOR_CONTROL
#define MOTOR_CONTROL

// some constrain
#define MAX_SPEED 100   
#define MAX_ANGLE 360
#define MIN_ANGLE 0
#define LEFT 0
#define RIGHT 1

// PID parameters
#define P_DATA 10
#define I_DATA 0.6 
#define D_DATA 1
#define ACCEPTTED_ERROR 10

int16_t constrain(int16_t input, int16_t min, int16_t max);
int16_t angleTransform(int16_t angle_ABS180);
void Moter_Move(uint16_t angle, bool if_related, uint16_t speed);

#endif // MOTOR_CONTROL