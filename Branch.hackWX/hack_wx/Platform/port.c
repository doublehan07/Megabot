/**
  ******************************************************************************
  * 初始化代码
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "port.h"
#include "ranging_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define DMA_JY901

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;
static __IO uint32_t uwTimingDelay, tickCnt;

/* Private function prototypes -----------------------------------------------*/
void RCC_init(void);
void GPIO_init(void);
void NVIC_init(void);
void EXTI_init(void);

void Systick_init(void);

/* Private functions ---------------------------------------------------------*/
//系统时钟配置
void RCC_init(void)
{
	ErrorStatus HSEStartUpStatus;
	
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();
 
	/* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
		//!!!WX-CODE!!!BEGIN
		/* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1);
    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);
    /*	ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    /* Configure PLLs *********************************************************/
    /* PLL1 configuration: PLLCLK = PLL * 6 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
    
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);
		//!!!WX-CODE!!!END

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08) {}
  }

	/* Enable Clock Security System(CSS): this will generate an NMI exception
     when HSE clock fails */
  RCC_ClockSecuritySystemCmd(ENABLE);
	
	RCC_GetClocksFreq(&RCC_Clocks);
	
	/* Enable AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* Enable GPIOs clocks */
	RCC_APB2PeriphClockCmd(
						RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
						RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);
}

//GPIO初始化
void GPIO_init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
}

//NVIC - 嵌套向量中断控制器配置
void NVIC_init(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure; //NVIC - 嵌套向量中断控制器	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//2+2模式
}

//EXTI - 外部中断配置
void EXTI_init(void)
{
//	EXTI_InitTypeDef EXTI_InitStructure;
}

/* 外设配置区 */
//Systick配置
//参数是SysTick的数目，默认应该是1ms/Tick
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

//每次SysTick会调用
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0)
  {  
    uwTimingDelay--;
  }
	
	if(tickCnt != 0)
	{
		tickCnt--;
	}
}

void SetTick(u32 tick)
{
	tickCnt = tick;
}

u32 GetTick(void)
{
	return tickCnt;
}

void Systick_init(void)
{
	//Set NVIC for SysTick
	NVIC_SetPriority (SysTick_IRQn, 0x05); //抢占优先级1，响应优先级1
	
	/* SysTick end of count event each 1ms */
	//RCC_GetClocksFreq(&RCC_Clocks); //系统时钟已经在时钟配置里获取，全局变量
	if (SysTick_Config(RCC_Clocks.HCLK_Frequency / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}	
}

//外设初始化
void Periph_init()
{
	Systick_init();
}

//系统总初始化
void Our_Sys_Init(void)
{
	RCC_init();
	GPIO_init();
	NVIC_init();
	EXTI_init();
	Periph_init();
}
