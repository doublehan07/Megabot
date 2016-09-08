#ifndef __PORT_H
#define __PORT_H

#include "stm32f10x.h"

// Motor Control
#define PHASE_PIN_LEFT					GPIO_Pin_5
#define PHASE_PIN_LEFT_2				GPIO_Pin_1
#define PHASE_PIN_RIGHT					GPIO_Pin_4
#define PHASE_PIN_RIGHT_2				GPIO_Pin_0
#define PHASE_GPIO							GPIOA

#define PWM_PIN_LEFT						GPIO_Pin_7
#define PWM_PIN_RIGHT						GPIO_Pin_6
#define PWM_TIM									TIM3
#define PWM_GPIO								GPIOA
#define PWM_PERIOD							2000

// Encoder
#define ENCODER_LEFT_B_IRQn			EXTI9_5_IRQn
#define ENCODER_LEFT_B					GPIO_Pin_7
#define ENCODER_LEFT_B_Source		GPIO_PinSource7
#define ENCODER_LEFT_B_Line			EXTI_Line7

#define ENCODER_LEFT_A_IRQn			EXTI2_IRQn
#define ENCODER_LEFT_A					GPIO_Pin_2
#define ENCODER_LEFT_A_Source		GPIO_PinSource2
#define ENCODER_LEFT_A_Line			EXTI_Line2

#define ENCODER_RIGHT_A_IRQn		EXTI1_IRQn
#define ENCODER_RIGHT_A					GPIO_Pin_1
#define ENCODER_RIGHT_A_Source	GPIO_PinSource1
#define ENCODER_RIGHT_A_Line		EXTI_Line1

#define ENCODER_RIGHT_B_IRQn		EXTI0_IRQn
#define ENCODER_RIGHT_B					GPIO_Pin_0
#define ENCODER_RIGHT_B_Source	GPIO_PinSource0
#define ENCODER_RIGHT_B_Line		EXTI_Line0

#define ENCODER_GPIO						GPIOB
#define ENCODER_PORT_SOURCE			GPIO_PortSourceGPIOB
#define ENCODER_SAVE_PERIOD			30

// USART Control
// BT COMMUNICATION USART
#define BT_USART_IRQn						USART1_IRQn
#define BT_USART_RX_Pin					GPIO_Pin_10
#define BT_USART_RX_GPIO				GPIOA
#define BT_USART_BaudRate				9600
#define BT_USART_Channel				USART1

// IMU USART
#define IMU_USART_IRQn					USART2_IRQn
#define IMU_USART_RX_Pin				GPIO_Pin_3
#define IMU_USART_RX_GPIO				GPIOA
#define IMU_USART_BaudRate			115200
#define IMU_USART_Channel				USART2

// WX COMMUNICATION USART
#define WX_USART_IRQn						USART3_IRQn
#define WX_USART_RX_Pin					GPIO_Pin_11
#define WX_USART_RX_GPIO				GPIOB
#define WX_USART_TX_Pin					GPIO_Pin_10
#define WX_USART_TX_GPIO				GPIOB
#define WX_USART_BaudRate				9600
#define WX_USART_Channel				USART3

#endif
