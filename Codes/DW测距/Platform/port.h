/**
  ******************************************************************************
  * 接口定义
  ******************************************************************************
  */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond
 * tick timer. */
#undef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC 											1000

//void printf2(const char *format, ...);

int writetospi_serial(uint16_t headerLength,
                             const uint8_t *headerBuffer,
                             uint32_t bodylength,
                             const uint8_t *bodyBuffer);

int readfromspi_serial(uint16_t	headerLength,
                              const uint8_t *headerBuffer,
                              uint32_t readlength,
                              uint8_t *readBuffer );

#define writetospi  												writetospi_serial
#define readfromspi												  readfromspi_serial

//SPI3 for DW1000
#define SPI_DW1000_RCC_CLOCK								RCC_APB1Periph_SPI3															
															
#define SPI_DW1000_PRESCALER								SPI_BaudRatePrescaler_4 //DW的SPI 20M是极限
#define SPI_DW1000													SPI3
#define SPI_DW1000_AF												GPIO_AF_SPI3
#define SPI_DW1000_GPIO											GPIOC
#define SPI_DW1000_CS												GPIO_Pin_15
#define SPI_DW1000_CS_GPIO									GPIOA
#define SPI_DW1000_SCK											GPIO_Pin_10
#define SPI_DW1000_MISO											GPIO_Pin_11
#define SPI_DW1000_MOSI											GPIO_Pin_12
#define SPI_DW1000_SCK_Source								GPIO_PinSource10
#define SPI_DW1000_MISO_Source							GPIO_PinSource11
#define SPI_DW1000_MOSI_Source							GPIO_PinSource12

#define DW1000_RSTn													GPIO_Pin_4
#define DW1000_RSTn_GPIO										GPIOB
#define DECARSTIRQ               					  GPIO_Pin_4
#define DECARSTIRQ_GPIO            				  GPIOB
#define DECARSTIRQ_EXTI           				  EXTI_Line4
#define DECARSTIRQ_EXTI_PORT      				  EXTI_PortSourceGPIOB
#define DECARSTIRQ_EXTI_PIN       				  EXTI_PinSource4
#define DECARSTIRQ_EXTI_IRQn      				  EXTI4_IRQn

#define DECAIRQ                 			  	  GPIO_Pin_2
#define DECAIRQ_GPIO          							GPIOD
#define DECAIRQ_EXTI            			 	    EXTI_Line2
#define DECAIRQ_EXTI_PORT       			  	  EXTI_PortSourceGPIOD
#define DECAIRQ_EXTI_PIN        			  	  EXTI_PinSource2
#define DECAIRQ_EXTI_IRQn        			 	    EXTI2_IRQn

#define port_SPI_DW1000_busy_sending()			(SPI_I2S_GetFlagStatus((SPI_DW1000),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPI_DW1000_no_data()						(SPI_I2S_GetFlagStatus((SPI_DW1000),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPI_DW1000_send_data(x)				SPI_I2S_SendData((SPI_DW1000),(x))
#define port_SPI_DW1000_receive_data()			SPI_I2S_ReceiveData(SPI_DW1000)
#define port_SPI_DW1000_disable()						SPI_Cmd(SPI_DW1000,DISABLE)
#define port_SPI_DW1000_enable()						SPI_Cmd(SPI_DW1000,ENABLE)
#define port_SPI_DW1000_set_chip_select()		GPIO_SetBits(SPI_DW1000_CS_GPIO,SPI_DW1000_CS)
#define port_SPI_DW1000_clear_chip_select()	GPIO_ResetBits(SPI_DW1000_CS_GPIO,SPI_DW1000_CS)

ITStatus EXTI_GetITEnStatus(uint32_t x);
#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)

void SPI_DW1000_ChangeRate(uint16_t scalingfactor);
void SPI_DW1000_ConfigFastRate(uint16_t scalingfactor);
void spi_DW1000_set_rate_low (void); //Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
void spi_DW1000_set_rate_high (void); //Set SPI rate as close to 20 MHz as possible for optimum performances.

void reset_DW1000(void);

//USART6 send distance data to upper controller
#define STM_TX															GPIO_Pin_6
#define STM_RX															GPIO_Pin_7
#define STM_UART_PORT												GPIOC
#define ENABLE_USART												GPIO_AF_USART6
#define AF_TX																GPIO_PinSource6
#define AF_RX																GPIO_PinSource7
#define USART_CHANNEL												USART6
#define USART_IRQN													USART6_IRQn
#define USART_RCC_CLOCK											RCC_APB2Periph_USART6

void Usart_TX_SendData(uint8_t *pointer, uint8_t length);

//#define port_GET_stack_pointer()		__get_MSP()

unsigned long portGetTickCnt(void);
#define portGetTickCount() 			portGetTickCnt()
extern __IO uint32_t uwTimingDelay;
void Delay(__IO uint32_t nTime);

void peripherals_init(uint32_t baudrate); //Initialise all peripherals.

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
