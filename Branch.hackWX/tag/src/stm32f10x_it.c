/**
******************************************************************************
* @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
* @author  MCD Application Team
* @version V3.5.0
* @date    08-April-2011
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "port.h"
__IO unsigned long time32_incr;    

uint8_t	HostCmdRxBuf[256];
uint16_t	HostCmdRxIndex;
extern uint8_t	SendBuf[];
extern uint8_t txIndex;
extern uint8_t SendLen;
extern uint8_t Flag_Uart_Send;


/** @addtogroup STM32F10x_StdPeriph_Template
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
* @brief  This function handles RTC global interrupt request.
* @param  None
* @retval None
*/
void RTCAlarm_IRQHandler(void)
{
  RTC_WaitForSynchro();
  if (RTC_GetITStatus(RTC_IT_ALR) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line17);
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
      PWR_ClearFlag(PWR_FLAG_WU);
    }
    RTC_ClearITPendingBit(RTC_IT_ALR);
    RTC_WaitForLastTask();
    
    LeaveStopMode();
    //RFWakeup();
  }
  
  return;
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
  TimingDelay_Increase();
  TimingDelay_Decrement();
  
}

void EXTI15_10_IRQHandler(void)
{
  
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
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
/**
* @brief  This function handles ScenSor  interrupt request.
* @param  None
* @retval None
*/
void EXTI3_IRQHandler(void)
{
  /* Clear EXTI Line 3 Pending Bit */
  EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI0_IRQHandler(void)
{
  /* Clear EXTI Line 0 Pending Bit */
  EXTI_ClearITPendingBit(DECAIRQ_EXTI);
}

void EXTI9_5_IRQHandler(void)
{
  
  EXTI_ClearITPendingBit(EXTI_Line6);
}


void USART1_IRQHandler(void)
{
  uint8_t	tmp;
    
  if((USART_GetITStatus(USART1, USART_IT_RXNE) == SET))
  { 
    tmp = USART_ReceiveData(USART1);
		
    if(HostCmdRxIndex == 256)
    {
      //Reset the packet index.
      HostCmdRxIndex = 0;
    }
    else
    {
      if(tmp == 0x08) 
      { //backspace      
        USART_SendData(USART1, 0x08);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, ' ');
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        USART_SendData(USART1, 0x08);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        
        HostCmdRxBuf[--HostCmdRxIndex] = '\0';
      }
      else
      {
        HostCmdRxBuf[HostCmdRxIndex++] = tmp;
        USART_SendData(USART1,tmp);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
      }
      if(tmp == 0x0d)
      {
        //
      }
    }
    
  }
}

void USART2_IRQHandler(void)
{
  uint8_t	tmp;
    
  if((USART_GetITStatus(USART2, USART_IT_RXNE) == SET))
  { 
    tmp = USART_ReceiveData(USART2);
		
    if(256 == HostCmdRxIndex)
    {
      //Reset the packet index.
      HostCmdRxIndex = 0;
    }
    else
    {
      if(tmp == 0x08) 
      { //backspace      
        USART_SendData(USART2, 0x08);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, ' ');
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        USART_SendData(USART2, 0x08);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        
        HostCmdRxBuf[--HostCmdRxIndex] = '\0';
      }
      else
      {
        HostCmdRxBuf[HostCmdRxIndex++] = tmp;
        USART_SendData(USART2,tmp);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
      }
      if(tmp == 0x0d)
      {
        
      }
    }
    
  }
}

uint8_t                 prev_char = 0;
uint8_t                 tmpIndex=0;
uint8_t                 header_ok=0;
extern uint8_t config_tinycon_flag;
extern uint8_t config_ok;
uint8_t datalen = 0;

void USART3_IRQHandler(void)
{  
  uint8_t tmp;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
  {
    tmp = USART_ReceiveData(USART3);
    if (config_tinycon_flag == 1)
    {
      //printf("%c",tmp);
      if ((tmp == 0x4f) && (config_ok == 0))
      {
        tmpIndex = 0;
        prev_char = tmp;
      }
      else if (prev_char == 0x4f)
      {
        if ((tmpIndex == 1) && (tmp == 0x4b))
        {
          config_ok = 1;
        }
        else
        {
          tmpIndex = 0;
          prev_char = 0;
        }
      }
      tmpIndex++;
    }
    else
    {
      if ((tmp == 0x57) && (header_ok == 0))
      {
        tmpIndex = 0;
        prev_char = tmp;
      }
      else if (prev_char == 0x57)
      {
        if ((tmpIndex == 1) && (tmp == 0x58))
        {
          header_ok = 1;
          
        }
        else
        {
          tmpIndex = 0;
          prev_char = 0;
        }
      }
      tmpIndex++;
    }
  }
}


void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    //irq_send_ccp();
    TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
  }
  
}


//串口1DMA方式发送中断
void DMA1_Channel4_IRQHandler(void)
{
  //if (DMA_GetITStatus(DMA1_IT_TC4)!= RESET)
  //{
  DMA_ClearFlag(DMA1_FLAG_TC4);
  //DMA_ClearITPendingBit(DMA1_IT_TC4);
  
  DMA_Cmd(DMA1_Channel4,DISABLE);
  //}
}

/**
* @}
*/


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
