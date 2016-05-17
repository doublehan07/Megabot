/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "port.h"
#include "motor_pcb_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
	//所有外设的初始化：时钟、IO、定时器等
  Our_Sys_Init();
	/* Insert 50 ms delay */
  Delay(10);
	//关于初始化：
	//如果一开始就可以做的尽量都放在开始时
	//如果有后续工作则在此处另外调用
	
	Motor_Init();
	Delay(5);
	Motor_If_Awake(1);
	Motor_If_FastDecay(0);
	
	// 1 means forward
	Motor_If_Forward(0, 1);
	Motor_If_Forward(1, 1);
	Motor_Set_Speed(0, 100);
	Motor_Set_Speed(1, 100);
	setSpeed(1000);
	
	while(1)
	{
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
