/**
  ******************************************************************************
  * Ö÷º¯Êý´úÂë
	* DW²â¾àÄ£¿éÅäÖÃ
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "port.h"
#include "ranging_api.h"

#include "initiator.h"
#include "usart2.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Channe2 , PRF64M , Preamble length = 1024 , Preamble code = 9 , 110k , standard SFD */
dwt_config_t config = {
  2,               /* Channel number. */
  DWT_PRF_64M,     /* Pulse repetition frequency. */
  DWT_PLEN_1024, 	 /* Preamble length. */
  DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
  9,               /* TX preamble code. Used in TX only. */
  9,               /* RX preamble code. Used in RX only. */
  0,         		   /* Use non-standard SFD (Boolean) */
  DWT_BR_6M8,      /* Data rate. */
  DWT_PHRMODE_STD, /* PHY header mode. */
  (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int main(void)
{
	//Start with board specific hardware init.
	Our_Sys_Init();
	Delay(5);
	Dwm1000_Init();
	USART2_INTERFACE_Configuration(115200);

  RESET_DW1000(); //Target specific drive of RSTn line into DW1000 low for a period.
  SPI_DW1000_set_rate_low();
  dwt_initialise(DWT_LOADUCODE); //Do not load any data from OTP.
  SPI_DW1000_set_rate_high();

  dwt_configure(&config); //Configure DW1000.

	// Apply antenna delay value.
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
	
	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //No frame filter will be used.
	
	//Set GPIO[3:0] as output, need inlude "deca_regs.h".
//	dwt_setGPIOdirection(GDM0, 0);
//	dwt_setGPIOdirection(GDM1, 0);
	dwt_setGPIOdirection(GDM2, 0);
//	dwt_setGPIOdirection(GDM3, 0);
	
//	dwt_setGPIOvalue(GDM0, GDP0);
//	dwt_setGPIOvalue(GDM1, GDP1);
	dwt_setGPIOvalue(GDM2, 0);
//	dwt_setGPIOvalue(GDM3, GDP3);
		
	 /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	
  while (1)
  {
//		dwt_setGPIOvalue(GDM2, 0);
//		Delay(5);
//		dwt_setGPIOvalue(GDM2, GDP2);
//		Delay(5);
		Initiator_Ranging(0x01);
		
		//Delay(1);
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
