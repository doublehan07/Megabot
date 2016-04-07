/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    21-October-2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		GPIO_ResetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5);
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/* Tick timer count. */
__IO uint32_t uwTimingDelay;
void SysTick_Handler(void)
{		
		TimingDelay_Decrement();
		ADC_PWM_Motor_Exec();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles USART1 Handler. For JY901.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART_JY901_CHANNEL, USART_IT_RXNE) != RESET)
	{	
		u8 tmp = USART_ReceiveData(USART_JY901_CHANNEL);
		ParseSerialData(tmp);
		if (findFirstData_JY901) //需要对齐数据头
		{
			if ((unsigned char)USART_ReceiveData(USART_JY901_CHANNEL) == 0x55)
			{
				findFirstData_JY901 = 0;
				//找到数据头处理函数
			}
		}
	}
	//USART不用手动清除标志位
}

/**
  * @brief  This function handles USART1_RX DMA Handler. For JY901.
  * @param  None
  * @retval None
  */
void DMA2_Stream2_IRQHandler(void)
{  
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
  {
		ParseDMAData();
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
  }
}

/**
  * @brief  This function handles USART6 Handler. For DW1000.
  * @param  None
  * @retval None
  */
void USART6_IRQHandler(void)
{
	static uint8_t flagI = 0;
	
	if(USART_GetITStatus(USART_DW1000_CHANNEL, USART_IT_RXNE) != RESET)			
	{		
		u8 tmp = USART_ReceiveData(USART_DW1000_CHANNEL);
		Parse_DW1000_Data(tmp);	
		if (findFirstData_DW1000) //需要对齐数据头
		{
			if (tmp == 0x0A)
			{
				flagI = 1;
			}
			else if(flagI && tmp == 0xCF)
			{
				flagI = 0;
				findFirstData_DW1000 = 0;
				//找到数据头处理函数
			}
		}		
	}
	//USART不用手动清除标志位
}

/**
  * @brief  This function handles USART6_RX DMA Handler. For DW1000.
  * @param  None
  * @retval None
  */
void DMA2_Stream1_IRQHandler(void)
{  
  /* Test on DMA Stream Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
  {
		
    /* Clear DMA Stream Transfer Complete interrupt pending bit */
    DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
