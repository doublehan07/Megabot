/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_IMU_H
#define __USART_IMU_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//For JY-901
#define BAUDRATE_JY901					115200
#define USART_JY901_CLOCK		RCC_APB2Periph_USART1
#define STM_JY901_TX						    GPIO_Pin_9
#define STM_JY901_RX						    GPIO_Pin_10
#define STM_JY901_PORT				    GPIOA
#define AF_JY901_TX							    GPIO_PinSource9
#define AF_JY901_RX							    GPIO_PinSource10
#define AF_JY901_CHANNEL				GPIO_AF_USART1
#define JY901_IRQn							        USART1_IRQn

/* Exported functions ------------------------------------------------------- */

#endif /* __USART_IMU_H */
