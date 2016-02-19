/**
  ******************************************************************************
  * 接口定义
  ******************************************************************************
  */
 
#include "deca_sleep.h"
#include "port.h"

#define rcc_init(x)						RCC_Configuration(x)
#define systick_init(x)				SysTick_Configuration(x)
#define interrupt_init(x)			NVIC_Configuration(x)
#define gpio_init(x)					GPIO_Configuration(x)
#define spi3_init(x)					SPI3_DW1000_Configuration(x)
#define usart6_init(x)				USART6_SENDING_DATA_Configration(x)

/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;

/* Internal functions prototypes. */
static void spi_peripheral_init(void);
static void usart_peripheral_init(uint32_t baudrate);

//System Configuration
unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

int No_Configuration(void)
{
	return -1;
}

int SysTick_Configuration(void)
{
	/* SysTick end of count event each 1ms */
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	if (SysTick_Config(RCC_Clocks.HCLK_Frequency / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);

	return 0;
}
int NVIC_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure; //EXTI - 外部中断
	NVIC_InitTypeDef NVIC_InitStructure; //NVIC - 嵌套向量中断控制器
	
	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //嵌套 = 是没有同级的情况。同级不能嵌套(i.e.被打断)

	//Config EXTI & NVIC used for DW1000
	/* Connect EXTI Line to GPIO Pin */
	SYSCFG_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
	
	//Config NVIC used for date transferring - USART6
	/* Interrupt while receiving data */
	USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, ENABLE); //接收到信息中断
	
	/* Enable and set USART6 Interrupt the the second lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = USART_IRQN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	return 0;
}

int RCC_Configuration(void)
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
		/* HSE=8MHz
		 * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz 						*/
		/****************************************************************/
		/* HCLK = SYSCLK = 72MHz - AHB */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);  
		/* PCLK2 = HCLK = 72MHz APB2 */
		RCC_PCLK2Config(RCC_HCLK_Div1); 
		/* PCLK1 = HCLK/2 = 36MHz APB1 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* Configure PLLs *********************************************************/
		/* PLLCLK = HSE(8M)/8*288/4 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE, 8, 288, 4, 8);
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

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(
						RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD , ENABLE);
	
	/* Enable SPI3 clock for DW1000*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	/* Enable USART6 clock for data transferring */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	
	/* Enable EXTI & NVIC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	
	return 0;
}

int GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure all unused GPIO port pins in Analog Input mode (floating input
	* trigger OFF), this will reduce the power consumption and increase the device
	* immunity against EMI/EMC */
	// Set all GPIO pins as analog inputs
//这样SW会无法下载程序的！以后再优化吧
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
//	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	//Enable GPIO used for DW1000
	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = DECAIRQ;
	GPIO_InitStructure.GPIO_Mode = 	GPIO_Mode_IN;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);
	
	//Enable GPIO used for data transferring - USART6
	GPIO_InitStructure.GPIO_Pin = STM_TX | STM_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	
	GPIO_Init(STM_UART_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(STM_UART_PORT, STM_TX, ENABLE_USART);
	GPIO_PinAFConfig(STM_UART_PORT, STM_RX, ENABLE_USART);

  return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));

  enablestatus =  EXTI->IMR & EXTI_Line;
  if (enablestatus != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

//DW1000 Configuration
void SPI_DW1000_ChangeRate(uint16_t scalingfactor)
{
	uint16_t tmpreg = 0;

	/* Get the SPI_DW1000 CR1 value */
	tmpreg = SPI_DW1000->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= scalingfactor;

	/* Write to SPI_DW1000 CR1 */
	SPI_DW1000->CR1 = tmpreg;
}

void spi_DW1000_set_rate_low (void) //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
{
    SPI_DW1000_ChangeRate(SPI_BaudRatePrescaler_16);
}


void spi_DW1000_set_rate_high (void) //Set SPI rate as close to 20 MHz as possible for optimum performances.
{
    SPI_DW1000_ChangeRate(SPI_BaudRatePrescaler_2);
}

void SPI_DW1000_ConfigFastRate(uint16_t scalingfactor)
{
	SPI_InitTypeDef SPI_InitStructure;

	SPI_I2S_DeInit(SPI_DW1000);

	// SPI_DW1000 Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI_DW1000, &SPI_InitStructure);

	// Enable SPI_DW1000
	SPI_Cmd(SPI_DW1000, ENABLE);
}

int SPI3_DW1000_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	SPI_I2S_DeInit(SPI_DW1000);

	// SPI_DW1000 Mode setup
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //
	//SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	//SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
	//SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_DW1000_PRESCALER;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI_DW1000, &SPI_InitStructure);

	// SPI_DW1000 SCK and MOSI pin setup
	GPIO_InitStructure.GPIO_Pin = SPI_DW1000_SCK|SPI_DW1000_MOSI|SPI_DW1000_MISO;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SPI_DW1000_GPIO, &GPIO_InitStructure);
	GPIO_PinAFConfig(SPI_DW1000_GPIO,SPI_DW1000_SCK_Source,SPI_DW1000_AF);
	GPIO_PinAFConfig(SPI_DW1000_GPIO,SPI_DW1000_MISO_Source,SPI_DW1000_AF);
	GPIO_PinAFConfig(SPI_DW1000_GPIO,SPI_DW1000_MOSI_Source,SPI_DW1000_AF);

	// SPI_DW1000 CS pin setup
	GPIO_InitStructure.GPIO_Pin = SPI_DW1000_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_Init(SPI_DW1000_CS_GPIO, &GPIO_InitStructure);

	// Disable SPI_DW1000 SS Output
	SPI_SSOutputCmd(SPI_DW1000, DISABLE);

	// Enable SPI_DW1000
	SPI_Cmd(SPI_DW1000, ENABLE);

	// Set CS high
	GPIO_SetBits(SPI_DW1000_CS_GPIO, SPI_DW1000_CS);

  return 0;
}

void reset_DW1000(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO used for DW1000 reset
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

	//drive the RSTn pin low
	GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

	//put the pin back to tri-state ... as input
	GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

  deca_sleep(2);
}

//USART6 for sending data
int USART6_SENDING_DATA_Configration(uint32_t baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART_CHANNEL, &USART_InitStructure);   
	
	USART_ITConfig(USART_CHANNEL, USART_IT_RXNE, ENABLE);	
	USART_ClearFlag(USART_CHANNEL, USART_FLAG_TC);
    
	USART_Cmd(USART_CHANNEL, ENABLE);
	
	return 0;
}	

//Peripheral init
static void spi_peripheral_init(void) //Initialise all SPI peripherals at once.
{
	spi3_init();
}

static void usart_peripheral_init(uint32_t baudrate) //Initialise all USART peripherals at once
{
	usart6_init(baudrate);
}

void peripherals_init (uint32_t baudrate) //Initialise all peripherals.
{
	rcc_init();
	systick_init();
	gpio_init();
	interrupt_init();
	spi_peripheral_init();
	usart_peripheral_init(baudrate);
}
