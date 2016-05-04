/*!
 * \file main.c
 * \copyright
 * Copyright (c) 2015 Fairchild Semiconductor Corporation or subsidiaries
 * worldwide. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. 	Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 2. 	Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 3. 	Neither the names of the copyright holders nor the names of their
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "fis1100_hal.h"
#include "fis1100_driver.h"

#include "stm32f30x_misc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_exti.h"

#include "stm32f30x_usart.h"

#include "assert.h"
#include "string.h"

//#define NEW_OIS_TEST

/* FIS1100 shield connection interface */
#define FIS_RST_PORT		GPIOB
#define FIS_RST_PIN			GPIO_Pin_11

#define FIS_CS_PORT			GPIOC
#define FIS_CS_PIN			GPIO_Pin_15

#define FIS_INT1_PORT		GPIOC
#define FIS_INT1_PIN		GPIO_Pin_12

#define FIS_INT2_PORT		GPIOC
#define FIS_INT2_PIN		GPIO_Pin_14

/* USART Communication boards Interface */
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_APBPERIPHCLOCK            RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_3                
#define USARTx_TX_GPIO_PORT              GPIOB                       
#define USARTx_TX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USARTx_TX_SOURCE                 GPIO_PinSource3
#define USARTx_TX_AF                     GPIO_AF_7

#define USARTx_RX_PIN                    GPIO_Pin_4               
#define USARTx_RX_GPIO_PORT              GPIOB                    
#define USARTx_RX_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define USARTx_RX_SOURCE                 GPIO_PinSource4
#define USARTx_RX_AF                     GPIO_AF_7

#define USARTx_TDR_ADDRESS               0x40004428
#define USARTx_TX_DMA_CHANNEL            DMA1_Channel7
#define USARTx_TX_DMA_FLAG_TC            DMA1_FLAG_TC7
#define USARTx_TX_DMA_FLAG_GL            DMA1_FLAG_GL7

#define DMAx_CLK                         RCC_AHBPeriph_DMA1

#define HEADER_SIZE			4
#define CRC_SIZE			2
#define DATA_SIZE			240
#define TX_BUFFER_SIZE		(HEADER_SIZE + DATA_SIZE + CRC_SIZE)
#define PAYLOAD_LENGTH		(DATA_SIZE + CRC_SIZE)

#define GYRO_DATA_LENGTH	6

#ifndef NEW_OIS_TEST
	#define FIS_INT_LINE		EXTI_Line12
	#define FIS_INT_PINSOURCE 	EXTI_PinSource12
	#define FIS_INT_TRIGGER 	EXTI_Trigger_Falling
#else 
	#define FIS_INT_LINE		EXTI_Line14
	#define FIS_INT_PINSOURCE 	EXTI_PinSource14
	#define FIS_INT_TRIGGER 	EXTI_Trigger_Rising
#endif

DMA_InitTypeDef DMA_InitStructure;

static struct Fis1100Hal m_fishal;		//FIS1100 hal structure required to init the FIS1100 driver

volatile uint8_t tx_buffer[2*TX_BUFFER_SIZE]; //buffer used to lower the rate of USART transaction
volatile uint8_t headerPacket[HEADER_SIZE] = {255, 254, 0, PAYLOAD_LENGTH};

static uint16_t pointer = 0; 	//pointer used to keep track of the USART buffer
static uint16_t crc = 0;		//simple 16bit crc 
int16_t data[3];				//global array used to monitor gyroscope registers value with SWD interface

/* \brief Implements a blocking microseconds software delay required from HAL
*/
void myDelayMicroseconds(uint32_t t_us){
	
	for (uint32_t n = 0; n < t_us; n++)
	{
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
	}
}

/**
* \brief Common function protoype used to handle different OIS data ready events 
*/
static void (*myReadRawOis)(uint8_t* data);

/** !
*	\brief Implements HAL SPI full duplex transaction used from HAL functions
*/
uint8_t SPI_ReadWriteByte(uint8_t data)
{
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) ;
	SPI_SendData8(SPI2, data);
	
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) ;
	return SPI_ReceiveData8(SPI2);
}

/** !
*	\brief Implements HAL I2C/SPI transaction used to write FIS1100 registers
*/
void myWriteFisData(const uint8_t* data, uint8_t dataLength)
{
	GPIO_ResetBits(FIS_CS_PORT, FIS_CS_PIN);
	
	for(int i=0;i<dataLength; i++)
	{
		SPI_ReadWriteByte(data[i]);
	}
	
	GPIO_SetBits(FIS_CS_PORT, FIS_CS_PIN);

}

/* \brief Implements HAL I2C/SPI transaction used to write FIS1100 registers
*
*/
void myReadFisData(uint8_t address, uint8_t* data, uint8_t dataLength)
{
	GPIO_ResetBits(FIS_CS_PORT, FIS_CS_PIN);
	
	SPI_ReadWriteByte(address);
		
	for(int i=0;i<dataLength; i++)
	{
		data[i] = SPI_ReadWriteByte(0xff);
	}
		
	GPIO_SetBits(FIS_CS_PORT, FIS_CS_PIN);

}

/** Implements reset function required from FIS1100 HAL
*/
void myReset(bool assert)
{
	if(assert)
		GPIO_SetBits(FIS_RST_PORT, FIS_RST_PIN); 	//fis1100 reset pin HIGH for assert == true 
	else
		GPIO_ResetBits(FIS_RST_PORT, FIS_RST_PIN); 	//fis1100 reset pin LOW for assert == false 
}

/** Monitors interrupt 1 line pin value
*/
bool myReadInt1(){
	
	if(GPIO_ReadInputDataBit(FIS_INT1_PORT, FIS_INT1_PIN)>0)
		return true;	//if int1 pin value is HIGH
	else
		return false;	// if int1 pin value is LOW

}

/** Monitors interrupt 2 line pin value
*/
bool myReadInt2(){
	if(GPIO_ReadInputDataBit(FIS_INT2_PORT, FIS_INT2_PIN)>0)
		return true; 	//if int2 pin value is HIGH
	else
		return false; 	//if int2 pin value is LOW
}

/**
  * \brief  Configures the USART Peripheral.
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  RCC_AHBPeriphClockCmd(DMAx_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_APBPERIPHCLOCK(USARTx_CLK, ENABLE);
  
  /* Connect PXx to USARTx_Tx */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  
  /* Connect PXx to USARTx_Rx */
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

  /* USARTx configuration ----------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = 1000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  /* NVIC configuration */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* DMA Configuration -------------------------------------------------------*/
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

/** !
* \brief Implements blocking transfer over USART
*/
void usart_send_data(uint8_t *data, uint16_t length)
{
	for(int i=0;i<length; i++)
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, (uint16_t)data[i]);
	}
}

/** !
* \brief Configure External Interrupt to handle FIS1100 data ready events and DMA tranfer complete
*/
void intConfigure()
{	
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, FIS_INT_PINSOURCE); 
	EXTI_InitStructure.EXTI_Line = FIS_INT_LINE;
	EXTI_InitStructure.EXTI_Trigger = FIS_INT_TRIGGER;
	
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI15_10 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/** !
* \brief Setup DMA to start tranferring a buffer of byters over USART
* \param memory address of the buffer to transfer
* \param number of bytes to tranfer
*/
void dmaStartFromAddr(uint32_t memoryAddr, uint16_t size)
{
	/* Configure the USART to send the command table */    
	/* DMA channel Tx of USART Configuration */
	DMA_DeInit(USARTx_TX_DMA_CHANNEL);
	
	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	DMA_ITConfig(USARTx_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = USARTx_TDR_ADDRESS;
	DMA_InitStructure.DMA_BufferSize = size;
	DMA_InitStructure.DMA_MemoryBaseAddr = memoryAddr;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(USARTx_TX_DMA_CHANNEL, &DMA_InitStructure);
	 
	/* Enable the USART DMA requests */
	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
      
	/* Clear the TC bit in the SR register by writing 0 to it */
	USART_ClearFlag(USARTx, USART_FLAG_TC);
      
	/* Enable the DMA USART Tx Channel */
	DMA_Cmd(USARTx_TX_DMA_CHANNEL, ENABLE);
	 
}

/** !
* \brief Handles UASRT-DMA transfer complete for the used DMA and Channel (DMA1, Channel 7)
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* Test on DMA1 Channel7 Transfer Complete interrupt */
  if(DMA_GetITStatus(DMA1_IT_TC7))
  {
	 /* Clear DMA1 Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits */
     DMA_ClearFlag(USARTx_TX_DMA_FLAG_GL);
	 //DMA_ClearITPendingBit(DMA1_IT_GL7);
	 
	 DMA_Cmd(USARTx_TX_DMA_CHANNEL, DISABLE);
      
      /* Disable the USART Tx DMA requests */
	 USART_DMACmd(USARTx, USART_DMAReq_Tx, DISABLE);
  }
}

/**
  * \brief  This function handles FIS1100 data ready interrupt (either on INT1/INT2)
  * The function is used to syncrhonously read gyroscope registers value, to build up the Tx buffer and to start the USART communication (DMA based)
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(FIS_INT_LINE) != RESET)
  {  
	  
	myReadRawOis((uint8_t *)&tx_buffer[pointer]);
	
	memcpy(&data[0],(uint8_t *)&tx_buffer[pointer], GYRO_DATA_LENGTH);
	
	for(int i=0; i<GYRO_DATA_LENGTH; i++)
		crc += tx_buffer[pointer+i];

	pointer += GYRO_DATA_LENGTH;
		
	if(pointer == TX_BUFFER_SIZE-CRC_SIZE)
	{
		crc = ~crc;
		memcpy((uint8_t *)&tx_buffer[TX_BUFFER_SIZE-CRC_SIZE],(uint8_t *)&crc,2);
		
		dmaStartFromAddr((uint32_t)&tx_buffer[0], TX_BUFFER_SIZE); 
		//usart_send_data((uint8_t *)&tx_buffer[0], TX_BUFFER_SIZE); //alternative blocking call 
		
		pointer += CRC_SIZE + HEADER_SIZE;
		crc = 0;
	}
	
	if(pointer == 2*TX_BUFFER_SIZE-CRC_SIZE)
	{
		crc = ~crc;
		memcpy((uint8_t *)&tx_buffer[2*TX_BUFFER_SIZE-CRC_SIZE],(uint8_t *)&crc,2);
		
		dmaStartFromAddr((uint32_t)&tx_buffer[TX_BUFFER_SIZE], TX_BUFFER_SIZE); 
		//usart_send_data((uint8_t *)&tx_buffer[TX_BUFFER_SIZE], TX_BUFFER_SIZE); //alternative blocking call 
		
		pointer = HEADER_SIZE;
		crc = 0;
	}
	
    /* Clear the EXTI line pending bit */
    EXTI_ClearITPendingBit(FIS_INT_LINE);
  }
}

/** !
* Platform specific SPI configuration
*/
void spiInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	//Enable SPI peripheral clk
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	//Setup GPIOs configuration 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Setup alternate function for SPI pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);
	
	//Configure the SPI peripheral as full duplex master
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* Initialize the FIFO threshold */
  	SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);
  
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) ;
		
	SPI_Cmd(SPI2, ENABLE);
	
	uint8_t dummy = SPI_ReadWriteByte(0xFF);
	
}

/** !
* \brief Platform specific hardware configuration for GPIOs and peripherals 
*/
void hwInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Enable GPIOs clk
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//FIS1100 CS
	GPIO_InitStructure.GPIO_Pin = FIS_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(FIS_CS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(FIS_CS_PORT, FIS_CS_PIN);
	
	//FIS1100 RST
	GPIO_InitStructure.GPIO_Pin = FIS_RST_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(FIS_RST_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(FIS_RST_PORT, FIS_RST_PIN);
	
	//FIS1100 INT1/INT2 GPIO configuration
	GPIO_InitStructure.GPIO_Pin = FIS_INT1_PIN | FIS_INT2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		
	GPIO_Init(FIS_INT1_PORT, &GPIO_InitStructure);
	
	//Communication peripheral init for SPI and USART
	spiInit();
	USART_Config();
	
}

/** !
 * \brief Setup FIS1100 driver and sensor configuration 
 * Fill-in the required Hardware Abstraction Layer (HAL) structure with platform specific functions and enable OIS mode
*/
void fis1100Configure()
{
	m_fishal.writeData = myWriteFisData;
	m_fishal.readData = myReadFisData;
	m_fishal.assertReset = myReset;
	m_fishal.int1Asserted = myReadInt1;
	m_fishal.int2Asserted = myReadInt2;
	m_fishal.delayMicroseconds = myDelayMicroseconds; 
	m_fishal.spiMode = true; //false for I2C
	
	Fis1100_init(&m_fishal);
	
	Fis1100_configureAccelerometer(AccRange_8g, AccOdr_1024Hz, AccUnit_ms2, Lpf_Disable);

#ifdef NEW_OIS_TEST
	myReadRawOis = Fis1100_readRawOisGyroscopeData; //function added to handle new OIS mode data
	Fis1100_configureGyroscope(GyrRange_1024dps, GyrOdr_8192Hz_LP, GyrUnit_rads, Lpf_Disable); //Gyroscope setup for low latency OIS mode. Note: gODR added to set new OIS mode
#else
	myReadRawOis = Fis1100_readRawGyroscopeData;
	Fis1100_configureGyroscope(GyrRange_1024dps, GyrOdr_8192Hz, GyrUnit_rads, Lpf_Disable); //Gyroscope setup for normal OIS mode
#endif

	Fis1100_enableSensors(FIS1100_CTRL7_GYR_ENABLE);  //enable Gyroscope only
}

static void SetSysClock(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
  
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/    
  /* Enable HSE */    
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
    
    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
    
   
    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
                                        RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    
    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }
    
    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    
    
    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
  }
}

int main()
{	
	SetSysClock(); //Set the clock speed to 72 MHz
	SystemCoreClockUpdate(); //Update SystemCoreClock variable value 
	
	//prepare USART buffer with header used for data alignment
	for(int i=0; i<HEADER_SIZE; i++)
	{
		tx_buffer[i] = headerPacket[i];
		tx_buffer[TX_BUFFER_SIZE+i] = headerPacket[i];
	}
	
	//init pointer to TX buffer at end of the header
	pointer = HEADER_SIZE;
	
	//configure GPIOs and peripherals to handle FIS1100 shield interface, SPI and USART 
	hwInit();	
	
	//configure HAL and FIS1100 configuration
	fis1100Configure();
	
	//configure interrupt 
	intConfigure();
	
	while(1) //infinite loop 
	{
	}
}














