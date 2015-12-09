/**
  ******************************************************************************
  * ˵����д��д��������������������
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Init.h"
#include "DW1000.h"

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
	//���ڳ�ʼ����
	//���һ��ʼ�Ϳ������ľ��������ڿ�ʼʱ
	//����к����������ڴ˴��������
	
	DW1000_Init();//���SPI��ʼ�����ò��ԣ����ܻῨ���ڴ˴�
	
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
