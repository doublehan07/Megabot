/**
  ******************************************************************************
  * @file    TIM_PWM_Output/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"
#include "common.h"
#include "motor.h"
#include "usart.h"
#include "strategy.h"
#include "map.h"

extern uint8_t isParsed , isFinished , toBeReceive , requestToBeSent;
char counter, lastReceived , currentReceived ,reverse;
extern char ReceivedPacket[23] , toBeParsedPack[23];
extern char receive_data[32];
extern struct RECEIVE ReceiveAI ;
extern struct SEND SendAI;
extern char OurLocation[2] , lastOurLocation[2];
//extern FunType fp;

volatile bool bkp_l = false, bkp_r = false, left1 = false, left2 = false, right2 = false, right1 = false, if_back = false;

/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
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
  {}
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
  {}
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
  {}
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
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

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

//检测在黑线还是白线，驱动电机
void EXTI15_10_IRQHandler(void) //11.44V
{
	static char back_counter = 0;
//	GPIO_ResetBits(GPIOE,GPIO_Pin_4) ;
//	if(EXTI_GetITStatus(EXTI_Line10) != RESET) //bkp-l
//	{
//		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_10) == SET) //上升沿
//		{
//			bkp_l = true;
//			//motor_turn_right(0,1000);
//		}
//		else //下降沿
//		{
//			bkp_l = false;
//		}
//		EXTI_ClearITPendingBit(EXTI_Line10);
//	}
//	if(EXTI_GetITStatus(EXTI_Line11) != RESET) //bkp-r
//	{
//		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_11) == SET) //上升沿
//		{
//			bkp_r = true;
//			//motor_turn_left(0,1000);
//		}
//		else //下降沿
//		{
//			bkp_r = false;
//		}
//		EXTI_ClearITPendingBit(EXTI_Line11);
//	}
//	if(bkp_l && bkp_r)
//	{
//		motor_backward(1000);
//		if_back = true;
//	}
	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET) //LEFT2
	{
		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_13) == SET) //上升沿
		{
			left2 = true;
			if(!if_back)
			{
				motor_turn_right(-250,750);
			}
		}
		else //下降沿
		{
			left2 = false;
		}
		EXTI_ClearITPendingBit(EXTI_Line13);
	}
	
	if(EXTI_GetITStatus(EXTI_Line12) != RESET) //LEFT1
	{
		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_12) == SET) //上升沿
		{
			left1 = true;
			if(!if_back && !left2)
			{
				motor_turn_right(-250,750);
			}
		}
		else //下降沿
		{
			left1 = false;
		}
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET) //RIGHT2
	{
		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_14) == SET) //上升沿
		{
			right2 = true;
			if(!if_back)
			{
				motor_turn_left(-250,750);
			}
		}
		else //下降沿
		{
			right2 = false;
		}
		EXTI_ClearITPendingBit(EXTI_Line14);
	}

	if(EXTI_GetITStatus(EXTI_Line15) != RESET) //RIGHT1
	{
		if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_15) == SET) //上升沿
		{
			right1 = true;
			if(!if_back && !right2)
			{
				motor_turn_left(-250,750);
			}
		}
		else //下降沿
		{
			right1 = false;
		}
		EXTI_ClearITPendingBit(EXTI_Line15);
	}
	
	if(!if_back && left1 && right1)
	{
		//slow_decay();
		motor_backward(1000);
		if_back = true;
	}
	if(!if_back && !left1 && left2 && right2 && !right1)
	{
		//slow_decay();
		motor_backward(1000);
		if_back = true;
	}
	if(!if_back && left1 && left2 && right2 && !right1)
	{
		motor_turn_right(-150,850);
	}
	if(!if_back && !left1 && left2 && right2 && right1)
	{
		motor_turn_left(-150,850);
	}
	//GPIO_SetBits(GPIOE,GPIO_Pin_4);
}

void USART2_IRQHandler(void)
{ 
   if(USART_GetFlagStatus(USART2,USART_IT_RXNE)==SET) 
    {   
      //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);    
#if 0			
      lastReceived = currentReceived ;
      currentReceived = USART_ReceiveData(USART2);
      if (currentReceived == 0x0A && lastReceived == 0x0D)
      {
        isFinished = 1;//看到结尾符
        toBeReceive = 1;//正在接收数据
      }
      if(toBeReceive == 1)
      {
        if(isFinished){
          isFinished = 0;
        }else{
          ReceivedPacket[counter] = currentReceived;
          
          counter++;
          if (counter == 22)
          {
            toBeReceive = 0;
            counter = 0;
            ReceivedPacket[22]=0x0A;
            isFinished = 1;
						parseReceivedPack(ReceivedPacket);
          }
        }
			}
#else
			lastReceived = currentReceived;
			currentReceived = USART_ReceiveData(USART2);
			ReceivedPacket[counter] = currentReceived;
			ReceivedPacket[23+(counter++)] = currentReceived;
			counter %= 23;
      if (currentReceived == 0x0A && lastReceived == 0x0D)
      {
        isFinished = 1;//看到结尾符
        toBeReceive = 1;//正在接收数据
      }
			if (isFinished)
			{
				if (counter == 0)
					parseReceivedPack(ReceivedPacket+counter);
				counter = 0;
				isFinished = 0;
			}
#endif
			USART_ClearITPendingBit(USART2,USART_IT_RXNE);//
		}
}

void UART5_IRQHandler(void)
{//接受惯导传来的姿态位置信息
	if(USART_GetFlagStatus(UART5,USART_IT_RXNE)==SET) 
    {   
			char ch = USART_ReceiveData(UART5);
			imu_rev_process(ch);
			//printf("%c",ch);
		}
}


void TIM2_IRQHandler(void) //轮询红外是否越界
{		
	
	static uint32_t back_counter = 0 , reverse_counter = 0, forward_counter = 0;
	static bool if_slowdown = false;
//	if(ReceiveAI.Status != 0x01)
//	{
//		motor_sleep();
//	}
//	else
//	{
//		motor_wake();
//	}
//	if(reverse){
//		motor_turn_left( 0 , 1000 );
//		reverse_counter++;
//		if(reverse_counter>6){
//			reverse = 0;
//			reverse_counter=0;
//		}
//	}
	
	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
//			if((GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_10) == SET && GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_11) == SET) || (bkp_l && bkp_r)) //严重两边碰线
//			{
//				
//			}
//			else if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_10) == SET && bkp_l) //严重左边碰线
//			{
//				
//			}
//			else if(GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_11) == SET && bkp_r) //严重右边碰线
//			{
//				
//			}
//			else
//			{
//				bkp_l = false;
//				bkp_r = false;
				if((GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_12) == SET && GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_15) == SET) 
					|| (GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_13) == SET && GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_14) == SET) 
					|| (left1 && right1) || (left2 && right2)) //两边碰线
				{
					if_slowdown = false;
				}
				else if((GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_13) == SET && left2) 
					|| (GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_12) == SET && left1)) //左边碰线
				{
					if_slowdown = false;
				}
				else if((GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_14) == SET && right2) 
					|| (GPIO_ReadInputDataBit (GPIOD,GPIO_Pin_15) == SET && right1)) //右边碰线
				{
					if_slowdown = false;
				}
				else
				{
					left1 = false;
					right1 = false;
					left2 = false;
					right2 = false;
					//KeepForward = true;
					//motor_forward(800);
					motor_forward(600);
					forward_counter++;
					if(!if_slowdown && forward_counter == 10)
					{
						//motor_turn_left(500,50);
						motor_forward(500);
						if_slowdown = true;
						forward_counter = 0;
					}
				}
			}

//		if(if_back)
//		{
//			if(back_counter == 0)
//			{
//				motor_turn_right(0,1000);
//			}
//			back_counter++;
//			if(back_counter == 250)
//			{
//				motor_forward(600);
//			}
//			if(back_counter == 350)
//			{
//				if_back = false;
//				back_counter = 0;
//			}
//		}

			if(if_back && ( //(!bkp_l && !bkp_r) && 
				((!left1 && !right1) && (!left2 && !right2))))
			{
				//motor_backward(1000);
				//if(back_counter == 5)
				//{
					if(goingIN())
					{
						motor_turn_left(-150,850);
					}
					else
					{
						motor_turn_right(-150,850);
					}
					//motor_turn_right(-150,850);
				//}
				back_counter++;
				if(back_counter >= 15)
				{
					if_back = false;
					back_counter = 0;
				}
				motor_forward(450);
			}

  //}
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void TIM5_IRQHandler(void){//不断对更新的接收包进行决策
  if(isParsed == 1){
		int i = 3;	
		isParsed=0;
		//checkWhetherToBeAttacked();
		//if(ReceiveAI.Target&(1<<ReceiveAI.ID) && (ReceiveAI.PropNow&0xF0)!=0){
			if(ReceiveAI.PropI && ReceiveAI.PropI<0x05 && (!ReceiveAI.PropII)){
				if(ReceiveAI.Target&(1<<ReceiveAI.ID) && (ReceiveAI.PropNow&0xF0)!=0){
				useDefendProp();
				}
				//for(;i>0;i++){sendPack(ReceiveAI.ID,ReceiveAI.ID,ReceiveAI.PropI);}
			}
			else if(ReceiveAI.PropI && ReceiveAI.PropI<0x05 && ReceiveAI.PropII && ReceiveAI.PropII<0x05){
				useDefendProp();
			}
		//useAttackProp();
			else if(ReceiveAI.PropI>0x04){
				for(;i>0;i--){sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropI);}
			}else if(ReceiveAI.PropII>0x04){
				for(;i>0;i--){sendPack(ReceiveAI.ID,getRivalID(),ReceiveAI.PropII);}
			}
		getOurLocation(OurLocation);
		//useTrapProp();
		if(ReceiveAI.WarningD != 0){
			reverse = 1;
		}else{
			reverse = 0;
		}
		//saveMap();
  }
	getProp();
	
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
}


