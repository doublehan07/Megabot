#include "infrared.h"

void Infrared_GPIO_Config(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  
    /* GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);
  //SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource12);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource15);
	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //±ßÑØ´¥·¢
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
  EXTI_Init(&EXTI_InitStructure);	

	NVIC_Config();
};

void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);   
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
  NVIC_Init(&NVIC_InitStructure); 
}

