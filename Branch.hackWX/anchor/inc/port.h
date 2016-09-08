/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_flash.h"
#include "deca_types.h"
  

/*****************************************************************************************************************//*
 * To enable Direct Memory Access for SPI set this option to (1)
 * This option will increase speed of spi transactions but it will use an extra RAM memory buffer
 */
#define DMA_ENABLE  (0)

/*****************************************************************************************************************//*
**/
#if (DMA_ENABLE == 1)
 #define writetospi     writetospi_dma
 #define readfromspi    readfromspi_dma
 void dma_init(void);
#else

 extern int writetospi_serial( uint16 headerLength,
                    const uint8 *headerBuffer,
                    uint32 bodylength,
                    const uint8 *bodyBuffer
                  );

 extern int readfromspi_serial( uint16 headerLength,
                     const uint8 *headerBuffer,
                     uint32 readlength,
                     uint8 *readBuffer );

 #define writetospi     writetospi_serial
 #define readfromspi    readfromspi_serial
#endif

#define SPIx_PRESCALER				SPI_BaudRatePrescaler_8

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_MISO					GPIO_Pin_6
#define SPIx_MOSI					GPIO_Pin_7


#define DW1000_RSTn                 GPIO_Pin_11
#define DW1000_RSTn_GPIO            GPIOA
#define DECAIRQ                     GPIO_Pin_0
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line0
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource0
#define DECAIRQ_EXTI_IRQn           EXTI0_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE



#define port_SPIx_busy_sending()		(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_TXE))==(RESET))
#define port_SPIx_no_data()				(SPI_I2S_GetFlagStatus((SPIx),(SPI_I2S_FLAG_RXNE))==(RESET))
#define port_SPIx_send_data(x)			SPI_I2S_SendData((SPIx),(x))
#define port_SPIx_receive_data()		SPI_I2S_ReceiveData(SPIx)
#define port_SPIx_disable()				SPI_Cmd(SPIx,DISABLE)
#define port_SPIx_enable()              SPI_Cmd(SPIx,ENABLE)
#define port_SPIx_set_chip_select()		GPIO_SetBits(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	GPIO_ResetBits(SPIx_CS_GPIO,SPIx_CS)

#define port_GET_stack_pointer()        __get_MSP()
#define port_GET_rtc_time()				RTC_GetCounter()
#define port_SET_rtc_time(x)			RTC_SetCounter(x)

#define port_GetEXT_IRQStatus()             NVIC_GetPendingIRQ(DECAIRQ_EXTI_IRQn) //EXTI_GetFlagStatus(DECAIRQ_EXTI) //

#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
//#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
#define port_DisableIRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableIRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckIRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)


void __weak process_deca_irq(void);


uint32_t portGetTickCnt(void);	

void SYS_RCC_INIT(void);
void DW1000_NVIC_INIT(void);
void DW1000_SPI_INIT(void);

void DMA_NVIC_INIT(void);
void USART_DMA_Tx_Init(uint8_t *data, uint16_t size);
void USART_DMA_Tx_Set(uint8_t *data, uint16_t size);
void USART_DMA_Rx_Init(uint8_t *data, uint16_t size);
void USART_DMA_Rx_Set(uint8_t *data, uint16_t size);


void TIMER_NVIC(void);
void TIMER_cfg(uint16_t time_ms);
void setRstHigh(void);
void SPI_ChangeRate(uint16_t scalingfactor);
void USART1_Configuration(void);
void USART1_Configuration_Remap(void);
void USART2_Configuration(void);
void USART3_Configuration(void);
void TimingDelay_Decrement(void);
void TimingDelay_Increase(void);
void Delay(__IO uint32_t nTime);

void RTC_Configuration();
void EnterStopMode(uint32_t s);
void LeaveStopMode(void);
void InitRF_PAPort(void);
void InitRF_LNAPort(void);
void EnablePA(void);
void DisablePA(void);
void EnableLNA(void);
void DisableLNA(void);
//LED.
void InitLED(void);
void SetLEDGreen(uint8_t status);  //1, on, 0:off
void SetLEDYellow(uint8_t status); //1, on, 0:off

#define portGetTickCount() 			portGetTickCnt()

void reset_DW1000(void);

extern uint8 WIFI_EN;
#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
