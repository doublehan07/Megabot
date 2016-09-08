#include "init.h"

static __IO uint16_t systickDelay;

void RCC_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(
						RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_USART1 | 
						RCC_APB2Periph_AFIO, ENABLE);
}

void NVIC_Group_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

void PWM_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = PHASE_PIN_LEFT | PHASE_PIN_RIGHT | PHASE_PIN_LEFT_2 | PHASE_PIN_RIGHT_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PHASE_GPIO, &GPIO_InitStructure);
	
	GPIO_SetBits(PHASE_GPIO, PHASE_PIN_LEFT | PHASE_PIN_RIGHT | PHASE_PIN_LEFT_2 | PHASE_PIN_RIGHT_2);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_LEFT | PWM_PIN_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_GPIO, &GPIO_InitStructure);
}

void PWM_TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInit(PWM_TIM,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
	
	TIM_Cmd(PWM_TIM, ENABLE);
}

void Encoder_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;		
	
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_LEFT_A_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_LEFT_B_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_RIGHT_A_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ENCODER_RIGHT_B_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void Encoder_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = 
						ENCODER_LEFT_A | ENCODER_LEFT_B | 
						ENCODER_RIGHT_A | ENCODER_RIGHT_B;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(ENCODER_GPIO, &GPIO_InitStructure);
}

void Encoder_EXTI_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	
	EXTI_ClearITPendingBit(ENCODER_LEFT_A_Line);
	EXTI_ClearITPendingBit(ENCODER_LEFT_B_Line);
	EXTI_ClearITPendingBit(ENCODER_RIGHT_A_Line);
	EXTI_ClearITPendingBit(ENCODER_RIGHT_B_Line);
	
	GPIO_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_LEFT_A_Source);
	GPIO_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_LEFT_B_Source);
	GPIO_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_RIGHT_A_Source);
	GPIO_EXTILineConfig(ENCODER_PORT_SOURCE, ENCODER_RIGHT_B_Source);
	
	EXTI_InitStructure.EXTI_Line = 
						ENCODER_LEFT_A_Line | ENCODER_LEFT_B_Line | 
						ENCODER_RIGHT_A_Line | ENCODER_RIGHT_B_Line;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_Init(&EXTI_InitStructure);
}

void BT_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = BT_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
}

void BT_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = BT_USART_RX_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BT_USART_RX_GPIO, &GPIO_InitStructure);
}

void BT_USART_Init(void)
{
	USART_InitTypeDef USART_InitStructure; 
  
	USART_InitStructure.USART_BaudRate = BT_USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	
	USART_Init(BT_USART_Channel, &USART_InitStructure); 
	
	USART_ITConfig(BT_USART_Channel, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(BT_USART_Channel, ENABLE);
}

void IMU_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = IMU_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
}

void IMU_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = IMU_USART_RX_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(IMU_USART_RX_GPIO, &GPIO_InitStructure);
}

void IMU_USART_Init(void)
{
	USART_InitTypeDef USART_InitStructure; 
  
	USART_InitStructure.USART_BaudRate = IMU_USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	
	USART_Init(IMU_USART_Channel, &USART_InitStructure); 
	
	USART_ITConfig(IMU_USART_Channel, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(IMU_USART_Channel, ENABLE);
}

void WX_NVIC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = WX_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
}

void WX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = WX_USART_RX_Pin; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(WX_USART_RX_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = WX_USART_TX_Pin; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(WX_USART_TX_GPIO, &GPIO_InitStructure);
}

void WX_USART_Init(void)
{
	USART_InitTypeDef USART_InitStructure; 
  
	USART_InitStructure.USART_BaudRate = WX_USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(WX_USART_Channel, &USART_InitStructure); 
	
	USART_ITConfig(WX_USART_Channel, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(WX_USART_Channel, ENABLE);
}

void SysTick_Init(void)
{
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
	NVIC_SetPriority(SysTick_IRQn, 0x0E);
}

void InitAll(void)
{
	RCC_Init();
	NVIC_Group_Init();
	
	PWM_GPIO_Init();
	PWM_TIM_Init();
	
	Encoder_NVIC_Init();
	Encoder_GPIO_Init();
	Encoder_EXTI_Init();
	
	BT_NVIC_Init();
	BT_GPIO_Init();
	BT_USART_Init();
	
	IMU_NVIC_Init();
	IMU_GPIO_Init();
	IMU_USART_Init();
	
	WX_NVIC_Init();
	WX_GPIO_Init();
	WX_USART_Init();
	
	SysTick_Init();
}

void SystickDelay_Decrement(void)
{
	if(systickDelay != 0)
	{
		systickDelay--;
	}
}

void Delay(__IO uint16_t nTime)
{
	if(nTime <= 0)
	{
		return;
	}
	systickDelay = nTime;
	while(systickDelay != 0);
}
