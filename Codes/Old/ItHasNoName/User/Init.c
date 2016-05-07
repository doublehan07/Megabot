/**
  ******************************************************************************
  * 初始化代码
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Init.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
void RCCInit(void);
void GPIOInit(void);
void TimerInit(void);
void NVIC_Configuration(void);

/* Private functions ---------------------------------------------------------*/
void PeriphInit(void)
{
	RCCInit();
	GPIOInit();
	TimerInit();
	NVIC_Configuration();
}

//系统时钟：最高84MHz
void RCCInit(void)
{
	RCC_DeInit();
	
	/* HCLK = SYSCLK = 84MHz */
	RCC_HCLKConfig(RCC_SYSCLK_Div1); 
	/* PCLK2 = HCLK = 84MHz */
	RCC_PCLK2Config(RCC_HCLK_Div1); 
	/* PCLK1 = HCLK/2 = 42MHz */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_2);
	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(ENABLE);

	/* PLLCLK = HSI(16M)/16*336/4 = 84 MHz */
	RCC_PLLConfig(RCC_PLLSource_HSI, 16, 336, 4, 6);
	/* Enable PLL */ 
	RCC_PLLCmd(ENABLE);	  

	/* Wait till PLL is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	
	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while(RCC_GetSYSCLKSource() != 0x08);
	
	/* SysTick end of count event each 10ms */
	RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
}

//此处可以只初始化通用IO，每个模块专用的IO留着各部分初始化时再做即可
void GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//可以使用位或同时初始化多个IO
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//GPIO_Mode_IN GPIO_Mode_OUT
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//GPIO_OType_OD
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL GPIO_PuPd_DOWN
  //GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TimerInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
  TIM_TimeBaseStructure.TIM_Period =99;//从0计数到99，即10kHz
  TIM_TimeBaseStructure.TIM_Prescaler = RCC_Clocks.HCLK_Frequency/1000000 - 1;
	//预分频为84，实际计时时钟为1MHz，RCC_Clocks是初始化后获得的时钟值，除多少就相当于分频后时钟是多少
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//更新时中断
  //TIM_Cmd(TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = RCC_Clocks.HCLK_Frequency/500000 - 1;//2us
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM5, DISABLE);
}

//中断配置
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//2+2模式
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);

  /*NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

