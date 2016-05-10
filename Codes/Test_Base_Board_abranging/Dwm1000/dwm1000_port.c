/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dwm1000_port.h"
#include "ranging_api.h"          
#include "deca_sleep.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// System tick 32 bit variable defined by the platform
extern __IO unsigned long time32_incr;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line) //Checks whether the specified EXTI line is enabled or not.
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
  return bitstatus; //The "enable" state of EXTI_Line (SET or RESET).
}

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

void SPI_DW1000_set_rate_low (void) //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
{
    SPI_DW1000_ChangeRate(SPI_BaudRatePrescaler_16);
}


void SPI_DW1000_set_rate_high (void) //Set SPI rate as close to 20 MHz as possible for optimum performances.
{
    SPI_DW1000_ChangeRate(SPI_BaudRatePrescaler_2);
}

void RESET_DW1000(void)
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

void SPI3_DW1000_Configuration(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable SPI3 clock for DW1000
	RCC_APB1PeriphClockCmd(SPI_DW1000_RCC_CLOCK, ENABLE);
	
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
}

void Dwm1000_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure; //EXTI - 外部中断
	NVIC_InitTypeDef NVIC_InitStructure; //NVIC - 嵌套向量中断控制器
	
	//Congigure SPI3
	SPI3_DW1000_Configuration();
	
	// Enable GPIO used as DECA IRQ for interrupt
	GPIO_InitStructure.GPIO_Pin = DECAIRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	//IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(DECAIRQ_GPIO, &GPIO_InitStructure);
	
	// Connect EXTI Line to GPIO Pin
	SYSCFG_EXTILineConfig(DECAIRQ_EXTI_PORT, DECAIRQ_EXTI_PIN);

	// Configure EXTI line
	EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Interrupt to the lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}
