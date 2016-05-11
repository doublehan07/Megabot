/**
  ******************************************************************************
  * ��ʼ������
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "port.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
//#define DMA_JY901

/* Private variables ---------------------------------------------------------*/
RCC_ClocksTypeDef RCC_Clocks;

static __IO uint32_t uwTimingDelay;

__IO uint8_t findFirstData_JY901 = 0;
__IO uint8_t findFirstData_DW1000 = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_init(void);
void GPIO_init(void);
void NVIC_init(void);
void EXTI_init(void);

void Systick_init(void);

void Usart_JY901_init(void);
void Usart_JY901_DMA_init(void);

void Usart_DW1000_init(void);
void Usart_DW1000_DMA_init(void);

/* Private functions ---------------------------------------------------------*/
//ϵͳʱ������
void RCC_init(void)
{
	ErrorStatus HSEStartUpStatus;

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

	RCC_GetClocksFreq(&RCC_Clocks);
	
	/* Enable EXTI & NVIC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
						RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD , ENABLE);
}

//GPIO��ʼ��
void GPIO_init(void)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
}

//NVIC - Ƕ�������жϿ���������
void NVIC_init(void)
{
	//NVIC_InitTypeDef NVIC_InitStructure; //NVIC - Ƕ�������жϿ�����	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//2+2ģʽ
}

//EXTI - �ⲿ�ж�����
void EXTI_init(void)
{
//	EXTI_InitTypeDef EXTI_InitStructure;
}

/* ���������� */
//Systick����
//������SysTick����Ŀ��Ĭ��Ӧ����1ms/Tick
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

//ÿ��SysTick�����
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  {  
    uwTimingDelay--;
  }
}

void Systick_init(void)
{
	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);
	
	//Set NVIC for SysTick
	NVIC_SetPriority (SysTick_IRQn, 0x05); //��ռ���ȼ�1����Ӧ���ȼ�1
	
	/* SysTick end of count event each 1ms */
	//RCC_GetClocksFreq(&RCC_Clocks); //ϵͳʱ���Ѿ���ʱ���������ȡ��ȫ�ֱ���
	if (SysTick_Config(RCC_Clocks.HCLK_Frequency / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}	
}

//Usart����
//USART1-JY901
void Usart_JY901_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; //NVIC - Ƕ�������жϿ�����
	
	/* Enable USART1 clock for JY901 */
	RCC_APB2PeriphClockCmd(USART_JY901_CLOCK, ENABLE);
	
	/* Set GPIOs for USART1-JY901 */
	GPIO_InitStructure.GPIO_Pin = STM_JY901_TX | STM_JY901_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(STM_JY901_PORT, &GPIO_InitStructure);
    
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_TX, AF_JY901_CHANNEL);
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_RX, AF_JY901_CHANNEL);
	
	/* Usart1 Config */
	USART_InitStructure.USART_BaudRate = BAUDRATE_JY901;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART_JY901_CHANNEL, &USART_InitStructure);   

	/* Set NVIC for JY901-USART1 */
	// Interrupt while receiving data
	USART_ITConfig(USART_JY901_CHANNEL, USART_IT_RXNE, ENABLE); //���յ���Ϣ�ж�
	
	//Enable and set USART1 Interrupt the the second lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = JY901_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART_JY901_CHANNEL, ENABLE);
}

//�����ʼ��
void Periph_init()
{
	Systick_init();
	Usart_JY901_init();
}

//ϵͳ�ܳ�ʼ��
void Our_Sys_Init(void)
{
	RCC_init();
	GPIO_init();
	NVIC_init();
	EXTI_init();
	Periph_init();
}
