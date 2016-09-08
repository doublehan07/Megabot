#include "main.h"

static u8 last_cmd = 0;
static u8 IS_TURNNING = 0;
static u8 IS_ARRIVE = 0;

float getAngleDiffRad(float angle1, float angle2)
{
	float angle_diff = angle1 - angle2;
	while(angle_diff > PI)
		angle_diff -= 2*PI;
	while(angle_diff < -PI)
		angle_diff += 2*PI;
	return angle_diff;
}

int main(void)
{
	InitAll();
	#ifdef CALIBRATE_ENCODER
	Delay(3000);
	Motor_Set_Direc(2, 1);
	Set_CCR_PID_2(500, 10000);
	#endif
	#ifdef CALIBRATE_STRAIGHT
	Delay(3000);
	Motor_Set_Direc(2, 1);
	Set_CCR_PID(500);
	#endif
  while (1)
  {
		if(GetIsID0())
		{
			u8 cmd = GetCmd();
			u8 speed = GetSpeed();
			if(speed == 0)
				cmd = 0;
			if(cmd == 0)
				speed = 0;
			if(cmd == last_cmd)
				Set_Speed_PID(speed);
			else
				switch(cmd)
				{
					case 0: Motor_Brake(2); Set_PID_Stop(); break;
					case 1: Motor_Brake(2); Set_Speed_PID(speed); Motor_Set_Direc(2, 1); break;
					case 2: Motor_Brake(2); Set_Speed_PID(speed); Motor_Set_Direc(2, 0); break;
					case 3: Motor_Brake(2); Set_Speed_PID(speed); Motor_Turn(0); break;
					case 4: Motor_Brake(2); Set_Speed_PID(speed); Motor_Turn(1); break;				
					default: break;
				}
			last_cmd = cmd;
		}
		#if (defined CALIBRATE_ENCODER) || (defined CALIBRATE_STRAIGHT)
		else if(last_cmd == 1)
		#else
		else
		#endif
		{
			struct Route route = GetRoute();
			float dis2target;
			if(route.angle_0 != 0)
				if(!SetAngleOffset(route.angle_0))
					continue;
			if(pow(route.x - route.target_x, 2) + pow(route.y - route.target_y, 2) == 0)
			{
				Motor_Brake(2);
				Set_PID_Stop();
				continue;
			}
			dis2target = sqrt(pow(route.x + Getdx() - route.target_x, 2) + pow(route.y + Getdy() - route.target_y, 2));
			if(dis2target < ARRIVE_DIS_THRESHOLD + IS_ARRIVE * 50)
			{
				float angle_diff = getAngleDiffRad(getAngleRad(), route.angle_0);
				IS_ARRIVE = 1;
				if(fabs(angle_diff) > TURNNING_ANGLE_THRESHOLD)
				{
					// Turn to angle_0
					Motor_Brake(2);
					if(angle_diff > PI / 6)
						Set_Speed_PID(TURNNING_SPEED_FAST);
					else
						Set_Speed_PID(TURNNING_SPEED_SLOW);
					if(angle_diff > 0)
						Motor_Turn(1);
					else
						Motor_Turn(0);
				}
				else
				{
					Motor_Brake(2);
					Set_PID_Stop();
				}
			}
			else
			{
				float angle_threshold = (dis2target > ANGLE_THRESHOLD_CHANGE_DIS) ? FAR_ANGLE_THRESHOLD : NEAR_ANGLE_THRESHOLD;
				float target_angle = atan2(route.target_y - route.y - Getdy(), route.target_x - route.x - Getdx());
				float angle_diff1 = getAngleDiffRad(getAngleRad(), target_angle);
				float angle_diff2 = getAngleDiffRad(getAngleRad() - PI, target_angle);
				u8 speed = (dis2target + 250) / 20;
				IS_ARRIVE = 0;
				speed = (speed > 30) ? 30:speed;
				if(IS_TURNNING)
					angle_threshold = TURNNING_ANGLE_THRESHOLD;
				if(fabs(angle_diff1) <= fabs(angle_diff2))
				{
					// Move forward
					if(fabs(angle_diff1) > angle_threshold)
					{
						// Turn to target_angle
						IS_TURNNING = 1;
						if(fabs(angle_diff1) > PI / 6)
							Set_Speed_PID(TURNNING_SPEED_FAST);
						else
							Set_Speed_PID(TURNNING_SPEED_SLOW);
						if(angle_diff1 > 0)
							Motor_Turn(1);
						else
							Motor_Turn(0);
					}
					else
					{
						IS_TURNNING = 0;
						Motor_Brake(2);
						Set_Speed_PID(speed);
						Motor_Set_Direc(2, 1);
					}
				}
				else
				{
					// Move backward
					if(fabs(angle_diff2) > angle_threshold)
					{
						// Turn to target_angle + PI
						IS_TURNNING = 1;
						if(fabs(angle_diff2) > PI / 6)
							Set_Speed_PID(TURNNING_SPEED_FAST);
						else
							Set_Speed_PID(TURNNING_SPEED_SLOW);
						if(angle_diff2 > 0)
							Motor_Turn(1);
						else
							Motor_Turn(0);	
					}
					else
					{
						IS_TURNNING = 0;
						Motor_Brake(2);
						Set_Speed_PID(speed);
						Motor_Set_Direc(2, 0);						
					}
				}
			}
		}
  }
}
