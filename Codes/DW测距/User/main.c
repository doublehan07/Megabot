/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
	* DW测距模块配置
  ******************************************************************************
  */
	
#include "main.h"

/* Channel2 , PRF16M , Preamble length = 64 , Preamble code = 3 , 6.8M , standard SFD , SFD = 8 symbol times long */
dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_16M,     /* Pulse repetition frequency. */
    DWT_PLEN_64,  	 /* Preamble length. */
    DWT_PAC8,    	   /* Preamble acquisition chunk size. Used in RX only. */
    3,               /* TX preamble code. Used in TX only. */
    3,               /* RX preamble code. Used in RX only. */
    0,         		   /* Use non-standard SFD (Boolean) */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (64 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

int main(void)
{
		uint32_t baudrate = 115200;
	
    /* Start with board specific hardware init. */
    peripherals_init(baudrate);

    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_DW1000_set_rate_low();
    dwt_initialise(DWT_LOADUCODE); //Do not load any data from OTP.
    spi_DW1000_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Apply antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	
		/* No frame filter will be used. */
		dwt_enableframefilter(DWT_FF_NOTYPE_EN);
	
		/* Set GPIO3-LED_BLUE, GPIO0-LED_RED as output*/
		dwt_setGPIOdirection(GDM0, 0);
		dwt_setGPIOdirection(GDM1, 0);
		dwt_setGPIOdirection(GDM2, 0);
		dwt_setGPIOdirection(GDM3, 0);

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
				Ranging_Stategy();
		}
}
