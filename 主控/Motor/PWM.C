#include "pwm.h"

//timer3 channel 1-2
#define PWM_MOTOR_PORT GPIOA
#define PWM_MOTOR_PIN_1  GPIO_Pin_6
#define PWM_MOTOR_PIN_2  GPIO_Pin_7


void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	//clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//PA6 PA7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//gpio intialize
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_InitStructure.GPIO_Pin = PWM_MOTOR_PIN_1 | PWM_MOTOR_PIN_2;
	GPIO_Init(PWM_MOTOR_PORT, &GPIO_InitStructure);

	//AF
	GPIO_PinAFConfig(PWM_MOTOR_PORT,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(PWM_MOTOR_PORT,GPIO_PinSource7,GPIO_AF_TIM3);

	//timer3 initialize
	
	//timer3 base initialize @5kHz
	TIM_TimeBaseStructure.TIM_Prescaler = 47;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 799;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x00;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	//timer3 channel 1 initial
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//timer3 channel 2 initial
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_Cmd(TIM3, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	
}
