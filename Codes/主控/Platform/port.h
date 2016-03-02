#ifndef __PORT_H
#define __PORT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//For Systick
#ifdef CLOCKS_PER_SEC
	#undef CLOCKS_PER_SEC
#endif
#define CLOCKS_PER_SEC					1000

//For JY-901
#define BAUDRATE_JY901					9600
#define USART_JY901_CHANNEL			USART1
#define USART_JY901_CLOCK				RCC_APB2Periph_USART1
#define STM_JY901_TX						GPIO_Pin_9
#define STM_JY901_RX						GPIO_Pin_10
#define STM_JY901_PORT					GPIOA
#define AF_JY901_TX							GPIO_PinSource9
#define AF_JY901_RX							GPIO_PinSource10
#define AF_JY901_CHANNEL				GPIO_AF_USART1
#define JY901_IRQn							USART1_IRQn

//For DW1000
#define BAUDRATE_DW1000					9600
#define USART_DW1000_CHANNEL		USART6
#define USART_DW1000_CLOCK			RCC_APB2Periph_USART6
#define STM_DW1000_TX						GPIO_Pin_6
#define STM_DW1000_RX						GPIO_Pin_7
#define STM_DW1000_PORT					GPIOC
#define AF_DW1000_TX						GPIO_PinSource6
#define AF_DW1000_RX						GPIO_PinSource7
#define AF_DW1000_CHANNEL				GPIO_AF_USART6
#define DW1000_IRQn							USART6_IRQn

/* Exported functions ------------------------------------------------------- */
void Our_Sys_Init(void);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#endif
