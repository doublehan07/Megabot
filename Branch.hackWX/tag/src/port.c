// -------------------------------------------------------------------------------------------------------------------
//
//  File: port.c -
//
//  Copyright 2011 (c) DecaWave Ltd, Dublin, Ireland.
//
//  All rights reserved.
//
//  Author: Ekawahyu Susilo, April 2011
//
// -------------------------------------------------------------------------------------------------------------------


#include "compiler.h"
#include "port.h"

//Fanghong, need add Macro to support this function. "_DLIB_FILE_DESCRIPTOR"
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//System tick 32 bit variable defined by the platform 
extern __IO unsigned long time32_incr;
static __IO uint32_t TimingDelay;
uint8 WIFI_EN = 0;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////
//void SYS_RCC_INIT()
//
//Enabled all the GPIO Port. .
//
/////////////////////////////////////////////////////////////////////////

void SYS_RCC_INIT()
{
  RCC_ClocksTypeDef RCC_ClockFreq;
#ifdef _LOW_POWER
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
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    /****************************************************************/
    /*		HSE=8MHz,                                           */
    /****************************************************************/
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
    
    
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}
    
    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSYSCLKSource() != 0x08){}
  }
#endif
  RCC_GetClocksFreq(&RCC_ClockFreq);
  
  /* Enable SPI1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  
  /* Enable SPI2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2|RCC_APB1Periph_TIM2, ENABLE);
  
  /* Enable GPIOs clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
                           ENABLE); 
  
  
  //Start System TICK
  SysTick_Config(SystemCoreClock / 1000);
  
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  //NVIC_SetPriority (SysTick_IRQn, 5);  
}

void DW1000_NVIC_INIT()
{  
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  //RCC_AHBPeriphClockCmd (RCC_AHBPeriph_GPIOC,ENABLE);//RCC_AHBPeriph_GPIOB,ENABLE);
  
  // Enable GPIO used as DECA IRQ for interrupt
  GPIO_InitStructure.GPIO_Pin = DECAIRQ;//GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL;
  GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);//GPIOA, &GPIO_InitStructure);
  
  
  /* Connect EXTI Line to GPIO Pin */
  //Mask by Fanghong, not need in STM32L Platform.
  GPIO_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);
  
  /* Configure EXTI line */
  EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;//EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
  EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  
  /* Enable and set EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;//EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DECAIRQ_EXTI_USEIRQ;
  
  NVIC_Init(&NVIC_InitStructure);
}

/////////////////////////////////////////////////////////////////////////
//void DW1000_SPI_INIT()
//
//Init the M3 SPI interface to Decewave. use the SPI1 on PortA.
//
/////////////////////////////////////////////////////////////////////////


void DW1000_SPI_INIT()
{
  SPI_InitTypeDef SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  SPI_I2S_DeInit(SPIx);
  
  // SPIx Mode setup
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
  //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
  //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  
  SPI_Init(SPIx, &SPI_InitStructure);
  
  // SPIx SCK and MOSI pin setup
  GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);
  
  // SPIx MISO pin setup
  GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  //GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_IPU;
  
  GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);
  
  // SPIx CS pin setup
  GPIO_InitStructure.GPIO_Pin = SPIx_CS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);
  
  // Disable SPIx SS Output
  SPI_SSOutputCmd(SPIx, DISABLE);
  
  // Enable SPIx
  SPI_Cmd(SPIx, ENABLE);
  
  // Set CS high
  GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS); 
  
}

void USART1_Configuration(void)
{
  USART_InitTypeDef 	USART_InitStructure;
  GPIO_InitTypeDef 	    GPIO_InitStructure;
  NVIC_InitTypeDef		NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  // USARTx TX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // USARTx RX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // USARTx setup
  USART_InitStructure.USART_BaudRate = 115200;//921600;//115200;
  //USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure); 		
  // Enables or Disables EXTI Interrupt to the specified priority 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  USART_Cmd(USART1, ENABLE);
}

void USART1_Configuration_Remap(void)
{
  USART_InitTypeDef 	USART_InitStructure;
  GPIO_InitTypeDef 	    GPIO_InitStructure;
  NVIC_InitTypeDef		NVIC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);
  
  // USARTx TX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  // USARTx RX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // USARTx setup
  USART_InitStructure.USART_BaudRate = 115200;//921600;//115200;
  //USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure); 		
  // Enables or Disables EXTI Interrupt to the specified priority 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  USART_Cmd(USART1, ENABLE);
}

void USART2_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  // USARTx TX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // USARTx RX pin setup
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // USARTx setup
  USART_InitStructure.USART_BaudRate = 115200;//921600;//115200;
  //USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART2, &USART_InitStructure); 		
  // Enables or Disables EXTI Interrupt to the specified priority 
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  USART_Cmd(USART2, ENABLE);
}

void USART3_Configuration(void)
{
  USART_InitTypeDef 	USART_InitStructure;
  GPIO_InitTypeDef 	    GPIO_InitStructure;
  NVIC_InitTypeDef		NVIC_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //memset((void*)&USART_InitStructure,0,sizeof(USART_InitTypeDef));
  // USARTx setup
  USART_InitStructure.USART_BaudRate = 115200;//921600;//115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART3, &USART_InitStructure);
  
  // Enables or Disables EXTI Interrupt to the specified priority 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  // Enable USARTx
  USART_Cmd(USART3, ENABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

void DMA_NVIC_INIT(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;     //usart 1  TX
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure); 
}

void USART_DMA_Tx_Init(uint8_t *data, uint16_t size)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_InitTypeDef DMA_InitStructure;   
  
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR); 
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)data;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE); 
  
#ifdef UT_TWR_CTL //interface_B modify
  USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
#endif
  
  //打开DMA
  USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  
  DMA_Cmd(DMA1_Channel4,ENABLE);
}

void USART_DMA_Tx_Set(uint8_t *data, uint16_t size)
{
  DMA_Cmd(DMA1_Channel4,DISABLE);  
  
  DMA1_Channel4->CMAR = (uint32_t)data; 
  DMA1_Channel4->CNDTR = size;  
  
  USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
  DMA_Cmd(DMA1_Channel4,ENABLE);  
}

void USART2_DMA_Tx_Init(uint8_t *data, uint16_t size)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_InitTypeDef DMA_InitStructure;   
  
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR); 
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)data;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE); 
  
#ifdef UT_TWR_CTL //interface_B modify
  USART_ITConfig(USART2,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
#endif
  
  //打开DMA
  USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  
  DMA_Cmd(DMA1_Channel4,ENABLE);
}

void USART2_DMA_Tx_Set(uint8_t *data, uint16_t size)
{
  DMA_Cmd(DMA1_Channel4,DISABLE);  
  
  DMA1_Channel4->CMAR = (uint32_t)data; 
  DMA1_Channel4->CNDTR = size;  
  
  USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
  DMA_Cmd(DMA1_Channel4,ENABLE);  
}

void USART_DMA_Rx_Init(uint8_t *data, uint16_t size)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_InitTypeDef DMA_InitStructure;   
  
  DMA_DeInit(DMA1_Channel5);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);  
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;  
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  
  DMA_InitStructure.DMA_BufferSize = size;  
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;  
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_Init(DMA1_Channel5,&DMA_InitStructure);  
  
#ifdef UT_TWR_CTL //interface_B modify
  USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);  
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
#endif
  
  //打开DMA  
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
  DMA_Cmd(DMA1_Channel5,ENABLE);  
}

void USART_DMA_Rx_Set(uint8_t *data, uint16_t size)
{
  DMA_Cmd(DMA1_Channel5,DISABLE);  
  
  DMA1_Channel5->CMAR = (uint32_t)data; 
  DMA1_Channel5->CNDTR = size;  
  
  USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
  DMA_Cmd(DMA1_Channel5,ENABLE);  
}

void TIMER_NVIC()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  //选择中断分组1
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
  //选择TIM2的中断通道
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  //抢占式中断优先级设置为0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //响应式中断优先级设置为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //使能中断
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIMER_cfg(uint16_t time_ms)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  //重新将Timer设置为缺省值
  TIM_DeInit(TIM2);
  //采用内部时钟给TIM2提供时钟源
  TIM_InternalClockConfig(TIM2);
  //预分频系数为36000-1，这样计数器时钟为72MHz/36000 = 2kHz
  TIM_TimeBaseStructure.TIM_Prescaler = 36000 - 1;
  //设置时钟分割
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  //设置计数器模式为向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = (time_ms*2)-1;
  //将配置应用到TIM2中
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
  
  //清除溢出中断标志
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  //禁止ARR预装载缓冲器
  TIM_ARRPreloadConfig(TIM2, DISABLE);
  //开启TIM2的中断
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  
  TIM_Cmd(TIM2,ENABLE);
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
}

void setRstHigh(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //Configure SPI pin: CS 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC,GPIO_Pin_10);
}

void reset_DW1000(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // Enable GPIO used for DW1000 reset
  GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);
  
  //drive the RSTn pin low
  GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);
  
  //put the pin back to tri-state ... as input
  GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);
  
  Delay(1);
}

void InitRF_PAPort(void)
{
  //PB3
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  // Enable GPIO used for DW1000 reset
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  Delay(1);
  GPIO_SetBits(GPIOB,GPIO_Pin_3);
}

//Init LNA power supply pin.
void InitRF_LNAPort(void)
{
  //PB5
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  // Enable GPIO used for DW1000 reset
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  //GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  Delay(1);
  GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

//enable PA Power supply.
void EnablePA(void)
{
  GPIO_SetBits(GPIOB,GPIO_Pin_3);
}

//Disable PA Power supply.
void DisablePA(void)
{ 
  GPIO_ResetBits(GPIOB,GPIO_Pin_3);
}

//enable LNA Power supply.
void EnableLNA(void)
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

//Disable LNA Power supply.
void DisableLNA(void)
{ 
  GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

int is_IRQ_enabled(void)
{
  return ((NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]& (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in milliseconds.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void TimingDelay_Increase()
{
  time32_incr++;
}

uint32_t portGetTickCnt(void)
{
  return time32_incr;
}
//////////////////////////////////////////////////////////////////////////////
// @brief  Retargets the C library printf function to the USART.
// @param  None
// @retval None
//////////////////////////////////////////////////////////////////////////////



/**
* @brief  Configures the RTC.
* @param  None
* @retval None
*/
void RTC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  EXTI_ClearITPendingBit(EXTI_Line17);
  EXTI_InitStructure.EXTI_Line = EXTI_Line17;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  BKP_ClearFlag();
  BKP_DeInit();
  RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {}
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);  
  RCC_RTCCLKCmd(ENABLE);
  RTC_WaitForSynchro();
  RTC_WaitForLastTask();
  RTC_ITConfig(RTC_IT_ALR, ENABLE);
  RTC_WaitForLastTask();
  
  RTC_SetPrescaler(38); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
  RTC_WaitForLastTask();
  
  RTC_SetCounter(0);
  RTC_WaitForLastTask();
  
}

void EnterStopMode(uint32_t s)
{
  GPIO_InitTypeDef GPIO_InitStructure;  
  ADC_Cmd(ADC1, DISABLE);
  DAC_Cmd(DAC_Channel_1,DISABLE);
  DAC_Cmd(DAC_Channel_2,DISABLE);  
  
  GPIO_DeInit (GPIOA );
  GPIO_DeInit (GPIOB );
  GPIO_DeInit (GPIOC );
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  |  RCC_APB2Periph_GPIOC ,
                         ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  |RCC_APB2Periph_GPIOC ,
                         DISABLE);
  
  PWR_ClearFlag(PWR_FLAG_WU);
  RTC_Configuration();
  RTC_ClearFlag(RTC_FLAG_SEC);
  while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
  RTC_SetAlarm(RTC_GetCounter() + s);
  RTC_WaitForLastTask();  
  RTC_WaitForSynchro();
  
  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

void LeaveStopMode(void)
{
  SYS_RCC_INIT();
  //USART3_Configuration();  
  USART1_Configuration();
  DW1000_SPI_INIT(); 
  //dma_init();
  //RTC_Configuration();
  DW1000_NVIC_INIT();	
  InitRF_PAPort();  
  //  StartHostInterface();
}

void InitLED(void)
{
  //PB9, PB7 
  GPIO_InitTypeDef GPIO_InitStructure;    
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_ResetBits(GPIOB,GPIO_Pin_7);
  GPIO_ResetBits(GPIOB,GPIO_Pin_6);    
}

//1, on, 0:off
void SetLEDGreen(uint8_t status)
{
  if(1 == status )
  {
    GPIO_SetBits(GPIOB,GPIO_Pin_7);
  }
  else
  {
    GPIO_ResetBits(GPIOB,GPIO_Pin_7);
  }
}
//1, on, 0:off
void SetLEDYellow(uint8_t status)
{
  if(1 == status )
  {
    GPIO_SetBits(GPIOB,GPIO_Pin_6);
  }
  else
  {
    GPIO_ResetBits(GPIOB,GPIO_Pin_6);
  }
}

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */  
  USART_SendData(USART1, (uint8_t) ch);
    
  /* Loop until transmit data register is empty */
  while (RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));
    
  return ch;
}

void init_modekey(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  
  // mode1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // mode2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // mode3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  // mode4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void read_modekey(void)
{
  WIFI_EN = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1);
}

