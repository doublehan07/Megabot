/**
  ******************************************************************************
  * ˵����д��д��������������������
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "port.h"
#include "motor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	//��������ĳ�ʼ����ʱ�ӡ�IO����ʱ����
	Our_Sys_Init();
	/* Insert 50 ms delay */
  Delay(50);
	//���ڳ�ʼ����
	//���һ��ʼ�Ϳ������ľ��������ڿ�ʼʱ
	//����к����������ڴ˴��������
	Motor_Init();
	Motor_Set(0);
	
  /* Infinite loop */
	Delay(1000);
  while (1)
  {
		/*
		Motor_Setspeed(MOTOR_LEFT, -40);
		Motor_Setspeed(MOTOR_RIGHT, -40);
		Delay(2000);
		Motor_Setspeed(MOTOR_LEFT, 40);
		Motor_Setspeed(MOTOR_RIGHT, 40);
		Delay(2000);
		*/
		Motor_SetAngle(90, 1);
		while(MOTOR_TURNNING_FLAG);
		Delay(2000);
		//Motor_Turnangle(-90, 1);
		//while(MOTOR_TURNNING_FLAG);
		//Delay(500);
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
