/**
  ******************************************************************************
  *   
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart_imu.h"
#include "motor_pcb_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//USART1-JY901

void Usart_JY901_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; //NVIC - 嵌套向量中断控制器
	
	/* Enable USART1 clock for JY901 */
	RCC_APB2PeriphClockCmd(USART_JY901_CLOCK, ENABLE);
	
	/* Set GPIOs for USART1-JY901 */
	GPIO_InitStructure.GPIO_Pin = STM_JY901_TX | STM_JY901_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(STM_JY901_PORT, &GPIO_InitStructure);
    
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_TX, AF_JY901_CHANNEL);
  GPIO_PinAFConfig(STM_JY901_PORT, AF_JY901_RX, AF_JY901_CHANNEL);
	
	/* Usart1 Config */
	USART_InitStructure.USART_BaudRate = BAUDRATE_JY901;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART_JY901_CHANNEL, &USART_InitStructure);   

	/* Set NVIC for JY901-USART1 */
	// Interrupt while receiving data
	USART_ITConfig(USART_JY901_CHANNEL, USART_IT_RXNE, ENABLE); //接收到信息中断
	
	//Enable and set USART1 Interrupt the the second lowest priority
	NVIC_InitStructure.NVIC_IRQChannel = JY901_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART_JY901_CHANNEL, ENABLE);
}
