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
__IO uint32_t time32_incr;
__IO uint32_t uwTimingDelay;
void SysTick_Handler(void)
{
    time32_incr++;
		if(time32_incr >= 0xFFFFFFFF) //防止50天后溢出
		{
				time32_incr = 0;
		}
		
		if (uwTimingDelay != 0x00) //Delay使用
		{  
				uwTimingDelay--;
		}
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
  * @}
  */ 

__IO uint8_t usart_rx_buffer[2];
__IO uint8_t Usart_RX_flag = RESET;
void USART6_IRQHandler(void) //串口收发
{
	static uint8_t temp = 0, start_check = RESET, receive_flag = RESET, end_check = RESET, i = 0;
	
		if(USART_GetITStatus(USART_CHANNEL, USART_IT_RXNE) != RESET)			
		{		
				//接收到上位机指令处理
				//与上位机通信格式：0x0A | 0xCF | 1-bit type | 1-bit id | 0xFC
				temp = USART_ReceiveData(USART_CHANNEL);

				if(temp == 0x0A)
				{
						start_check = SET;
				}
				else if(start_check && temp == 0xCF)
				{
						start_check = RESET;
						receive_flag = SET;
						i = 0;
				}
				else if(receive_flag && i < 2)
				{
						usart_rx_buffer[i] = temp;
						i++;
						if(i >= 2)
						{
								receive_flag = RESET;
								end_check = SET;
						}
				}
				else if(end_check)
				{
						if(temp == 0xFC)
						{
								Usart_RX_flag = SET;
						}
				}
		}
		//USART不用手动清除标志位
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
