/**
  ******************************************************************************
  * ˵����д��д��������������������
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Init.h"
#include "Motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

int main(void)
{
	//��������ĳ�ʼ����ʱ�ӡ�IO����ʱ����
	PeriphInit();
	/* Insert 50 ms delay */
  Delay(5);
	
	//MOTOR Init
	Motor_State_Conf(MOTOR_SLEEP);
	Motor_Decay_Conf(MOTOR_ALL, MOTOR_SLOW_DECAY_L);
	Delay(5); //50ms delay
	Motor_State_Conf(MOTOR_AWAKE);
	
  /* Infinite loop */
  while (1)
  {
		
  }
}

//������SysTick����Ŀ��Ĭ��Ӧ����10ms/Tick
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

//ÿ��SysTick�����
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  {  
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
