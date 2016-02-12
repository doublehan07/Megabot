#include "pid.h"

unsigned long lastTime;
double Input, Output, Setpoint;
double etSum, lastErr;
double kp, ki, kd;
//output = kp*et + ki*etSum + kd*det;
void PidFunction()
{   
	/*How long since we last calculated*/   
	// unsigned long now = millis();   				//得到当前时间
	// double timeChange = (double)(now - lastTime);   //得到当前时间与上次时间之间的间隔
	// /*Compute all the working error variables*/   	//
	// double et = Setpoint - Input;   				//反馈值与输入值的差值 e(t) = 比例
	// etSum += (et* timeChange);   					//差值*时间间隔乘积累加 = 积分  理论是零
	// double dEt = (et - lastErr) / timeChange;   	//差值-上一次差值 = 微分
	// /*Compute PID Output*/   						//
	// Output = kp * et + ki * etSum + kd * dEt;  		//输出 = 比例 + 积分 + 微分
	// /*Remember some variables for next time*/   
	// lastErr = et;   								//下次循环: 当前比例成为过去比例
	// lastTime = now;									//下次循环: 当前时间成为过去时间
} 

void ParaSet(double Kp, double Ki, double Kd)		//设置比例、积分、微分的系数
{   
	kp = Kp;   
	ki = Ki;   
	kd = Kd;
}