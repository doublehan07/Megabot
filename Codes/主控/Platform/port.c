/**
  ******************************************************************************
  * 初始化代码
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "port.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Systick_init(x)				Systick_Configuration(x)
#define Usart_JY901_init(x)		Usart1_Configuration(x)
#define Usart_DW1000_init(x)	Usart6_Configuration(x)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
static __IO uint32_t uwTimingDelay;

/* Private function prototypes -----------------------------------------------*/
void RCC_init(void);
void GPIO_init(void);
void NVIC_init(void);
void EXTI_init(void);

/* Private functions ---------------------------------------------------------*/
//系统时钟配置
void RCC_init(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;

	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(ENABLE);
		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/****************************************************************/
		/* HSE = 8MHz
		 * HCLK = 168MHz, PCLK2 = 84MHz, PCLK1 = 42MHz 									*/
		/****************************************************************/
		/* HCLK = SYSCLK = 168MHz - AHB */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);  
		/* PCLK2 = HCLK/2 = 84MHz APB2 */
		RCC_PCLK2Config(RCC_HCLK_Div2); 
		/* PCLK1 = HCLK/4 = 42MHz APB1 */
		RCC_PCLK1Config(RCC_HCLK_Div4);

		/* Configure PLLs *********************************************************/
		/* PLLCLK = HSE(8M) / 8 * 336 / 2 = 168MHz */
		//RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
		//RCC_PLLConfig(RCC_PLLSource_HSE, 8, 240, 2, 5);
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 192, 2, 4);
		
		/* Enable PLL */ 
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08){}
	}

	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	/* Enable EXTI & NVIC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	
	/* Enable USART1 clock for JY901 */
	RCC_APB2PeriphClockCmd(USART_JY901_CLOCK, ENABLE);
	
	/* Enable USART6 clock for DW1000 */
	RCC_APB2PeriphClockCmd(USART_DW1000_CLOCK, ENABLE);
}

//GPIO初始化
void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Set GPIOs for USART1-JY901 */
	GPIO_InitStructure.GPIO_Pin = STM_JY901_TX | STM_JY901_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(STM_JY901_PORT, &GPIO_InitStructure);
    
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_TX, AF_JY901_CHANNEL);
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_RX, AF_JY901_CHANNEL);	
	
	/* Set GPIOs for USART6-DW1000 */
	GPIO_InitStructure.GPIO_Pin = STM_DW1000_TX | STM_DW1000_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(STM_DW1000_PORT, &GPIO_InitStructure);
    
  GPIO_PinAFConfig(STM_DW1000_PORT, AF_DW1000_TX, AF_DW1000_CHANNEL);
  GPIO_PinAFConfig(STM_DW1000_PORT, AF_DW1000_RX, AF_DW1000_CHANNEL);	
}

//NVIC - 嵌套向量中断控制器配置
void NVIC_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; //NVIC - 嵌套向量中断控制器
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//2+2模式
	
	//Set NVIC for SysTick
	NVIC_SetPriority (SysTick_IRQn, 0x05); //抢占优先级1，响应优先级1
	
	//Set NVIC for JY901-USART1
	/* Interrupt while receiving data */
	USART_ITConfig(USART_JY901_CHANNEL, USART_IT_RXNE, ENABLE); //接收到信息中断
	
	/* Enable and set USART6 Interrupt the the second lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = JY901_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	//Set NVIC for DW1000-USART6
	/* Interrupt while receiving data */
	USART_ITConfig(USART_DW1000_CHANNEL, USART_IT_RXNE, ENABLE); //接收到信息中断
	
	/* Enable and set USART6 Interrupt the the second lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DW1000_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

//EXTI - 外部中断配置
void EXTI_init(void)
{
//	EXTI_InitTypeDef EXTI_InitStructure;
}


//Systick配置
void Systick_Configuration(void)
{
	/* SysTick end of count event each 1ms */
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	if (SysTick_Config(RCC_Clocks.HCLK_Frequency / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
}

//参数是SysTick的数目，默认应该是1ms/Tick
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

//每次SysTick会调用
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  {  
    uwTimingDelay--;
  }
}


//Usart配置
//USART1-JY901
void Usart1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = BAUDRATE_JY901;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART_JY901_CHANNEL, &USART_InitStructure);      
	USART_Cmd(USART_JY901_CHANNEL, ENABLE);
}

//USART6-DW1000
void Usart6_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = BAUDRATE_DW1000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART_DW1000_CHANNEL, &USART_InitStructure);       
	USART_Cmd(USART_DW1000_CHANNEL, ENABLE);
}

//外设初始化
void Periph_init()
{
	Usart_JY901_init();
	Usart_DW1000_init();
}

//系统总初始化
void Our_Sys_Init(void)
{
	RCC_init();
	Systick_init();
	GPIO_init();
	Periph_init();
	NVIC_init();
	EXTI_init();
	ADC_PWM_Motor_Init();
}
