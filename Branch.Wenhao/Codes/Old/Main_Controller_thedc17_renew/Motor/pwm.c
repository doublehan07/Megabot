/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t PWM_Period = 1000; //PWM Max Value
u8 PWMInited = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//Motor Configuration
void Timer1_PWM_Configuration(void)
{
	//Set Timer1 as PWM output timer for motor controlling - ENABLE_LEFT / ENABLE_RIGHT
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	//定时器初始化
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; 							//定时器预分频 APB2 /(TIM_Prescaler + 1) = 1M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = PWM_Period - 1; 					//自动重装值 (TIM_Period + 1) = 1k
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//设置时钟分割: TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	//计时输出
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择模式PWM1，小于比较值时有效
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//TIM1CH1-TIM1CH4失能
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;		//TIM1CH1N-TIM1CH4N使能
	TIM_OCInitStructure.TIM_Pulse = PWM_Period;												//每次捕获的比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;					//有效电平为低电平（由于使用的是反相输出）
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 				//输出同相,TIM_OCNPolarity_High时输出反相
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;			//TIM1CH1-TIM1CH4输出状态
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;			//TIM1CH1N-TIM1CH4N输出状态

	TIM_OC2Init(TIM1, &TIM_OCInitStructure); //TIM2N
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); //TIM3N

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void Timer3_Counter_Configuration(void)
{
	//Set Timer3 as Clock output timer for encoder counting
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//开启时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//定时器初始化
	TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1; 							//定时器预分频 APB2 /(TIM_Prescaler + 1) = 1M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 10 - 1; 					//自动重装值 (TIM_Period + 1) = 100k, 10us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//设置时钟分割: TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//更新时中断
	TIM_Cmd(TIM3, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Motor_EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	SYSCFG_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_LEFT_A_Source);
	SYSCFG_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_RIGHT_A_Source);
	
	EXTI_InitStructure.EXTI_Line = ENCODER_LEFT_A_Line;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = ENCODER_RIGHT_A_Line;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_LEFT_A_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_RIGHT_A_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_Init(&NVIC_InitStructure);
}

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable GPIO used for controlling motor
	GPIO_InitStructure.GPIO_Pin = PHASE_LEFT | PHASE_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_Init(MOTOR_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENABLE_LEFT | ENABLE_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(MOTOR_GPIO, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(MOTOR_GPIO, ENABLE_LEFT_Source, ENABLE_TIM);
	GPIO_PinAFConfig(MOTOR_GPIO, ENABLE_RIGHT_Source, ENABLE_TIM);
	Timer1_PWM_Configuration();
	
	//Enable Timer3
	Timer3_Counter_Configuration();
	
	//Enable GPIO used for Encoder
	GPIO_InitStructure.GPIO_Pin = ENCODER_LEFT_A | ENCODER_LEFT_B | ENCODER_RIGHT_A | ENCODER_RIGHT_B;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_Init(ENCODER_GPIO, &GPIO_InitStructure);
	Motor_EXTI_Configuration();
	
	PWMInited = 1;
}
