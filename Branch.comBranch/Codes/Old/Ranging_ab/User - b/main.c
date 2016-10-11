/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
	* TOA-接收端
  ******************************************************************************
  */

#include "main.h"

/* Channel2 , PRF16M , Preamble length = 64 , Preamble code = 3 , 6.8M , standard SFD , SFD = 8 symbol times long */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_16M,     /* Pulse repetition frequency. */
    DWT_PLEN_64,   	 /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    3,               /* TX preamble code. Used in TX only. */
    3,               /* RX preamble code. Used in RX only. */
    0,         		   /* Use non-standard SFD (Boolean) */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (64 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

//Target ID | Our ID | Times | TimeStamp | CRC | CRC
#define OurID	0x02
static uint8_t tx_reply_msg[] = {0x00, OurID, 0x01, 0x0D, 0x0A};
static uint8_t tx_final_msg[] = {0x00, OurID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};

/* Buffer to store received response message. */
#define RX_BUF_LEN 17
#define TS1_FIELD_INDEX 3
#define TS2_FIELD_INDEX 7
#define TS3_FIELD_INDEX 11
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
#define TimeStampLength 17
static uint64_t rx_DeviceB_1;
static uint64_t tx_DeviceB_2;
static uint64_t rx_DeviceB_3;

static uint32_t tx_DeviceA_1;
static uint32_t rx_DeviceA_2;
static uint32_t tx_DeviceA_3;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof = 0.0;
static double Treply1 = 0.0;
static double Treply2 = 0.0;
static double Tround1 = 0.0;
static double Tround2 = 0.0;
static double Tprop = 0.0;
static double distance = 0.0;
static uint16_t distance_cm = 0;

int main(void)
{
		uint32_t baudrate = 9600;
	
		/* Start with board specific hardware init. */
    peripherals_init(baudrate);

    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_DW1000_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_DW1000_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);

    /* Apply antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	
		/* No frame filter will be used. */
		dwt_enableframefilter(DWT_FF_NOTYPE_EN);
		
		/* Set GPIO3-LED_BLUE, GPIO0-LED_RED as output*/
//		dwt_setGPIOdirection(GDM0, 0);
//		dwt_setGPIOdirection(GDM3, 0);

    /* Loop forever responding to ranging requests. */
    while (1)
    {
			Receptor_Communication();
			
			/* Debug purpose. Test for ranging speed. */
//			dwt_setGPIOvalue(GDM3, GDP3);
//			Delay(100); //100ms
//			dwt_setGPIOvalue(GDM3, 0);
    }
}

void Receptor_Communication(void)
{
	uint8_t TargetID;
	
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);

	/* Activate reception immediately. */
	dwt_rxenable(0);

	/* Eeception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ };

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32_t frame_len;

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected message from Source Device(TargetID). */
		if(rx_buffer[0] == OurID && rx_buffer[2] == 0x01)
		{
			uint32_t resp_tx_time;
			uint8_t i;
			
			TargetID = rx_buffer[1];

			/* Retrieve poll reception timestamp. */
			rx_DeviceB_1 = get_rx_timestamp_u64(); //读取rx_deviceB_first_msg时间戳

			/* Set send time for response. */
			resp_tx_time = (rx_DeviceB_1 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_reply_msg时间戳
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(3700);
			
			//修改tx_reply_msg
			tx_reply_msg[0] = TargetID;

			/* Write and send the response message. */
			dwt_writetxdata(sizeof(tx_reply_msg), tx_reply_msg, 0);
			dwt_writetxfctrl(sizeof(tx_reply_msg), 0);
			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			/* We assume that the transmission is achieved correctly */
			//Transmisson Error Handler()
	
			/* Reception of a frame or error/timeout. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
			{ };

			if (status_reg & SYS_STATUS_RXFCG)
			{
				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is the expected message from Source Device(TargetID). */
				if(rx_buffer[0] == OurID && rx_buffer[1] == TargetID && rx_buffer[2] == 0x02)
				{
					int64_t tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					tx_DeviceB_2 = get_tx_timestamp_u64(); //读取tx_reply_msg时间戳
					rx_DeviceB_3= get_rx_timestamp_u64(); //读取rx_final_msg时间戳

					/* Get timestamps embedded in the final message. */
					tx_DeviceA_1 = 0;
					for(i = 0; i < 4; i++)
					{
						tx_DeviceA_1 += rx_buffer[TS1_FIELD_INDEX + i] << (i * 8);
					}
					
					rx_DeviceA_2 = 0;
					for(i = 0; i < 4; i++)
					{
						rx_DeviceA_2 += rx_buffer[TS2_FIELD_INDEX + i] << (i * 8);
					}
					
					tx_DeviceA_3 = 0;
					for(i = 0; i < 4; i++)
					{
						tx_DeviceA_3 += rx_buffer[TS3_FIELD_INDEX + i] << (i * 8);
					}

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. */					
					Treply1 = (double)(tx_DeviceB_2 - rx_DeviceB_1);
					Treply2 = (double)(tx_DeviceA_3 - rx_DeviceA_2);
					Tround1 = (double)(rx_DeviceA_2 - tx_DeviceA_1);
					Tround2 = (double)(rx_DeviceB_3 - tx_DeviceB_2);
					
					Tprop = (Treply1 * Treply2 - Tround2 * Tround1) / (Treply1 + Treply2 + Tround1 + Tround2);
					Tprop = Tprop > 0 ? Tprop : -Tprop;
					
					tof_dtu = (int64_t)Tprop;
					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					distance_cm = (uint16_t)(distance * 100);
					
					/* Debug purpose. Test for getting correct distance value speed. */
//					dwt_setGPIOvalue(GDM0, GDP0);
												
					/* Send distance to Source Device */
					//修改tx_final_msg
					for(i = 0; i < 8; i++)
					{
						tx_final_msg[TS1_FIELD_INDEX + i] = (uint8_t)tof_dtu;
						tof_dtu >>= 8;
					}
					tx_final_msg[0] = TargetID;
					
					dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
					dwt_writetxfctrl(sizeof(tx_final_msg), 0);
					dwt_starttx(DWT_START_TX_IMMEDIATE);
				}
			}
			else
			{
				/* Clear RX error events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
				
				/* Debug purpose. Test for getting correct distance value speed. */
//				dwt_setGPIOvalue(GDM0, 0);
			}
		}
	}
	else
	{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		
		/* Debug purpose. Test for getting correct distance value speed. */
//		dwt_setGPIOvalue(GDM0, 0);
	}
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}
