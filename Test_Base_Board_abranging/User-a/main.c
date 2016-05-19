/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
	* DW测距模块配置
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "ranging_api.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Channel2 , PRF16M , Preamble length = 64 , Preamble code = 3 , 6.8M , standard SFD , SFD = 8 symbol times long */
//dwt_config_t config = {
//  2,               /* Channel number. */
//  DWT_PRF_16M,     /* Pulse repetition frequency. */
//  DWT_PLEN_64,  	 /* Preamble length. */
//  DWT_PAC8,    	   /* Preamble acquisition chunk size. Used in RX only. */
//  3,               /* TX preamble code. Used in TX only. */
//  3,               /* RX preamble code. Used in RX only. */
//  0,         		   /* Use non-standard SFD (Boolean) */
//  DWT_BR_6M8,      /* Data rate. */
//  DWT_PHRMODE_STD, /* PHY header mode. */
//  (64 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
//};
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* Use non-standard SFD (Boolean) */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int main(void)
{
	/* Start with board specific hardware init. */
	Our_Sys_Init();
	Delay(5);
	Dwm1000_Init();

  RESET_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
  SPI_DW1000_set_rate_low();
  dwt_initialise(DWT_LOADUCODE); //Do not load any data from OTP.
  SPI_DW1000_set_rate_high();

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Apply antenna delay value. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
	
	/* No frame filter will be used. */
	dwt_enableframefilter(DWT_FF_NOTYPE_EN);
	
	/* Set GPIO[3:0] as output, need inlude "deca_regs.h". */
	dwt_setGPIOdirection(GDM0, 0);
	dwt_setGPIOdirection(GDM1, 0);
	dwt_setGPIOdirection(GDM2, 0);
	dwt_setGPIOdirection(GDM3, 0);
	
	dwt_setGPIOvalue(GDM0, GDP0);
	
  /* Loop forever initiating ranging exchanges. */
  while (1)
  {
//		dwt_setGPIOvalue(GDM1, GDP1);
//		dwt_setGPIOvalue(GDM2, GDP2);
//		dwt_setGPIOvalue(GDM3, GDP3);
		Initiator_Communication(0x02);
		Delay(5);
			
		/* Debug purpose. Test for ranging speed. */
//		dwt_setGPIOvalue(GDM3, GDP3);
//		Delay(100); //100ms
//		dwt_setGPIOvalue(GDM3, 0);
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
