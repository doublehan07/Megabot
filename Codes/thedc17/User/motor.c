#include "motor.h"


extern  __IO uint16_t ADC1ConvertedValue[2];
//void motor_config( FunType fpoint ){
void motor_config(){
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	//fp = fpoint ;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

  //ADC1_CH4_CH5_DMA_Config();
  //TIM2_Config();
}

//调节占空比控制电机速度,可供上层函数调用
void motor_setSpeed(uint32_t speed) {
  TIM1->CCR1= speed > 1000 ? 1000 : speed;
  TIM1->CCR2= speed > 1000 ? 1000 : speed;
}

void motor_setLeftSpeed(int32_t speed) 
{
	if(speed < 0)
	{
		speed = -speed;
	}
  TIM1->CCR1 = speed > 1000 ? 1000 : speed;
}
void motor_setRightSpeed(int32_t speed) 
{
	if(speed < 0)
	{
		speed = -speed;
	}
  TIM1->CCR2 = speed > 1000 ? 1000 : speed;
}

void motor_setDirection(uint32_t direction) {
  //结合惯导实现方向pid调节

}

void motor_wake(void){
  //nSLEEP1_on();
  GPIO_SetBits(GPIOA,GPIO_Pin_6) ;
  //nSLEEP2_on();
  GPIO_SetBits(GPIOB,GPIO_Pin_1) ;
}

void motor_sleep(void){
  //nSLEEP1_off();
  GPIO_ResetBits(GPIOA,GPIO_Pin_6) ;
  //nSLEEP2_off();
  GPIO_ResetBits(GPIOB,GPIO_Pin_1) ;
}

//enable
// void motor_enable(void){

// }
//brake
void motor_brake(void){
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
}

void motor_left_forward(void){
  //PHASE1_off();
  GPIO_ResetBits(GPIOA,GPIO_Pin_7) ;
//	//nSLEEP2_off();
//  GPIO_ResetBits(GPIOB,GPIO_Pin_1) ;
}

void motor_right_forward(void){
  //PHASE2_off();
  GPIO_ResetBits(GPIOE,GPIO_Pin_12) ;
//	//nSLEEP1_off();
//  GPIO_ResetBits(GPIOA,GPIO_Pin_6) ;
}

void motor_left_backward(void){
  //PHASE1_on();
  GPIO_SetBits(GPIOA,GPIO_Pin_7) ;
//	//nSLEEP2_off();
//  GPIO_ResetBits(GPIOB,GPIO_Pin_1) ;
}

void motor_right_backward(void){
   //PHASE2_on();
  GPIO_SetBits(GPIOE,GPIO_Pin_12) ;
//	//nSLEEP1_off();
//  GPIO_ResetBits(GPIOA,GPIO_Pin_6) ;
}

void motor_forward(uint32_t speed) 
{
	motor_left_forward();
	motor_right_forward();
	motor_setSpeed(speed);
}

void motor_backward(uint32_t speed) 
{
	motor_left_backward();
	motor_right_backward();
	motor_setSpeed(speed);
}

uint16_t get_left_adcVal(void){
  return ADC1ConvertedValue[0];
}

uint16_t get_right_adcVal(void){
  return ADC1ConvertedValue[1];
}

void motor_turn_left(int speed , int delta_speed)
{
	if(speed - delta_speed < 0)
	{
		if(speed + delta_speed > 0)
		{
			motor_left_backward();
			motor_right_forward();
			motor_setLeftSpeed(delta_speed - speed);
			motor_setRightSpeed(speed + delta_speed);
		}
		else
		{
			motor_left_backward();
			motor_right_backward();
			motor_setLeftSpeed(delta_speed - speed);
			motor_setRightSpeed(speed + delta_speed);
		}
	}
	else
	{
		if(speed + delta_speed > 0)
		{
			motor_left_forward();
			motor_right_forward();
			motor_setLeftSpeed(speed - delta_speed);
			motor_setRightSpeed(speed + delta_speed);
		}
		else
		{
			motor_left_forward();
			motor_right_backward();
			motor_setLeftSpeed(speed - delta_speed);
			motor_setRightSpeed(speed + delta_speed);
		}
	}
}

void motor_turn_right(int speed , int delta_speed)
{
	if(speed - delta_speed < 0)
	{
		if(speed + delta_speed > 0)
		{
			motor_left_forward();
			motor_right_backward();
			motor_setLeftSpeed(speed + delta_speed);
			motor_setRightSpeed(delta_speed - speed);
		}
		else
		{
			motor_left_backward();
			motor_right_backward();
			motor_setLeftSpeed(speed + delta_speed);
			motor_setRightSpeed(delta_speed - speed);
		}
	}
	else
	{
		if(speed + delta_speed > 0)
		{
			motor_left_forward();
			motor_right_forward();
			motor_setLeftSpeed(speed + delta_speed);
			motor_setRightSpeed(speed - delta_speed);
		}
		else
		{
			motor_left_backward();
			motor_right_forward();
			motor_setLeftSpeed(speed + delta_speed);
			motor_setRightSpeed(speed - delta_speed);
		}
	}
}

void left_fast_decay(void){
	GPIO_ResetBits(GPIOC,GPIO_Pin_4) ;
}
void right_fast_decay(void){
	GPIO_ResetBits(GPIOE,GPIO_Pin_13) ;
}
void left_slow_decay(void){
	GPIO_SetBits(GPIOC,GPIO_Pin_4) ;
}
void right_slow_decay(void){
	GPIO_SetBits(GPIOE,GPIO_Pin_13) ;
}
void slow_left_high(void){
	GPIO_SetBits(GPIOB,GPIO_Pin_0) ;
}
void slow_right_high(void){
	GPIO_SetBits(GPIOE,GPIO_Pin_15) ;
}
void slow_left_low(void){
	GPIO_ResetBits(GPIOB,GPIO_Pin_0) ;
}
void slow_right_low(void){
	GPIO_ResetBits(GPIOE,GPIO_Pin_15) ;
}
void fast_decay(void){
	motor_left_forward();
	motor_right_forward();
	left_fast_decay();
	right_fast_decay();
	motor_setSpeed(0);
}
void slow_decay(void){
	left_slow_decay();
	right_slow_decay();
	slow_left_low();
	slow_right_low();
	motor_setSpeed(0);
}


