/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DWM1000_PORT_H
#define __DWM1000_PORT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
int writetospi_serial(uint16_t headerLength,
                             const uint8_t *headerBuffer,
                             uint32_t bodylength,
                             const uint8_t *bodyBuffer);

int readfromspi_serial(uint16_t	headerLength,
                              const uint8_t *headerBuffer,
                              uint32_t readlength,
                              uint8_t *readBuffer );
															
unsigned long portGetTickCnt(void);
															
#define portGetTickCount() 									portGetTickCnt()

#define writetospi													writetospi_serial
#define readfromspi													readfromspi_serial

//SPI3 for DW1000
#define SPI_DW1000_RCC_CLOCK								RCC_APB2Periph_SPI1															
															
#define SPI_DW1000_PRESCALER								SPI_BaudRatePrescaler_16 //DW的SPI 20M是极限
#define SPI_DW1000													SPI1
#define SPI_DW1000_AF												GPIO_AF_SPI1
#define SPI_DW1000_CS												GPIO_Pin_4							//(407ZG)PA4-NSS
#define SPI_DW1000_CS_GPIO									GPIOA
#define SPI_DW1000_SCK											GPIO_Pin_5							//(103C8)PA5-SCK
#define SPI_DW1000_MISO											GPIO_Pin_6							//(103C8)PA6-MISO
#define SPI_DW1000_MOSI											GPIO_Pin_7							//(103C8)PA7-MOSI
#define SPI_DW1000_GPIO											GPIOA
#define SPI_DW1000_SCK_Source								GPIO_PinSource5
#define SPI_DW1000_MISO_Source							GPIO_PinSource6
#define SPI_DW1000_MOSI_Source							GPIO_PinSource7

#define DW1000_RSTn													GPIO_Pin_11
#define DW1000_RSTn_GPIO										GPIOA
//#define DECARSTIRQ               					  GPIO_Pin_4
//#define DECARSTIRQ_GPIO            				  GPIOB
//#define DECARSTIRQ_EXTI           				  EXTI_Line4
//#define DECARSTIRQ_EXTI_PORT      				  GPIO_PortSourceGPIOB
//#define DECARSTIRQ_EXTI_PIN       				  GPIO_PinSource4
//#define DECARSTIRQ_EXTI_IRQn      				  EXTI4_IRQn							//(103C8)EXTI_Line4

#define DECAIRQ                 			  	  GPIO_Pin_0
#define DECAIRQ_GPIO          							GPIOB
#define DECAIRQ_EXTI            			 	    EXTI_Line0
#define DECAIRQ_EXTI_PORT       			  	  GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN        			  	  GPIO_PinSource0
#define DECAIRQ_EXTI_IRQn        			 	    EXTI0_IRQn							//(103C8)EXTI_Line2

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

/* Exported functions ------------------------------------------------------- */
//DW1000
void SPI_DW1000_ChangeRate(uint16_t scalingfactor);
void SPI_DW1000_ConfigFastRate(uint16_t scalingfactor);

#endif /* __DWM1000_PORT_H */
