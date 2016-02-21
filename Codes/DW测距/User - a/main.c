/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
	* TOA-发送端
  ******************************************************************************
  */
	
#include "main.h"

/* Channel2 , PRF16M , Preamble length = 64 , Preamble code = 3 , 6.8M , standard SFD , SFD = 8 symbol times long */
static dwt_config_t config = {
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

//Target ID | Our ID | Time | CRC | CRC
#define OurID	0x01
static uint8_t tx_request_begin_range_msg[] = {0x00, OurID, 0x00, 0x00, 0x00};
static uint8_t tx_second_range_msg[] = {0x00, OurID, 0x00, 0x00, 0x00};

/* Buffer to store received response message. */
#define RX_BUF_LEN 5
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
#define TimeStampLength 5
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance = 0;
static uint16_t distance_cm;

int main(void)
{
		uint32_t baudrate = 9600;
	
    /* Start with board specific hardware init. */
    peripherals_init(baudrate);

    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_DW1000_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_DW1000_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
	
		/* No frame filter will be used. */
		dwt_enableframefilter(DWT_FF_NOTYPE_EN);

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
			distance_cm = Initiator_Communication(0x02);
			//distance = distance_cm / 100.0; debug - purpose
		}
}

uint16_t Initiator_Communication(uint8_t TargetID)
{
	uint16_t dist = 0;
	
 /* Set expected response's delay and timeout. 
  * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. 
	*/
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	
	/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
	
	/* Write frame data to DW1000 and prepare transmission. */	
	tx_request_begin_range_msg[0] = TargetID;
	
	dwt_writetxdata(sizeof(tx_request_begin_range_msg), tx_request_begin_range_msg, 0);
	dwt_writetxfctrl(sizeof(tx_request_begin_range_msg), 0);

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame 
	 * is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	/* We assume that the transmission is achieved correctly */
	//Transmisson Error Handler()
	
	/* Reception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ };

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32_t frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from Target Device(TargetID). */
		if(rx_buffer[0] == TargetID)
		{
			uint32_t final_tx_time;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64(); //读取tx_request_begin_range_msg时间戳
			resp_rx_ts = get_rx_timestamp_u64(); //读取rx_first_response_range_msg时间戳

			/* Compute final message transmission time. */
			final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_second_range_msg时间戳
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			//把Tround1和Treply2告诉Target Device(Treply2加上antenna delay)
			final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 
			
			//修改tx_second_range_msg

			/* Write and send final message. See NOTE 7 below. */
			dwt_writetxdata(sizeof(tx_second_range_msg), tx_second_range_msg, 0);
			dwt_writetxfctrl(sizeof(tx_second_range_msg), 0);
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

				/* Check that the frame is the expected response from Target Device(TargetID). */
				if(rx_buffer[0] == TargetID)
				{
					//get distance
					dist = 0;		
				}
			}
			else
			{
				/* Clear RX error events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			}
		}
	}
	else
	{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	}

	/* Execute a delay between ranging exchanges. */
	deca_sleep(RNG_DELAY_MS);
	
	return dist;
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
    int i;
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
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts)
{
    uint8_t i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
