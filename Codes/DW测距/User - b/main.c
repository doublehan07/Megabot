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

//Target ID | Our ID | Time | CRC | CRC
#define OurID	0x02
static uint8_t tx_reply_msg[] = {0x00, OurID, 0x00, 0x00, 0x00};
static uint8_t tx_final_msg[] = {0x00, OurID, 0x00, 0x00, 0x00};

/* Buffer to store received response message. */
#define RX_BUF_LEN 5
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
#define TimeStampLength 5
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance = 0;
static uint16_t distance_cm;

int main(void)
{
		uint32_t baudrate = 9600;
	
		/* Start with board specific hardware init. */
    peripherals_init(baudrate);

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for 
		 * optimum performance. */
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

    /* Loop forever responding to ranging requests. */
    while (1)
    {
			distance_cm = Receptor_Communication();
    }
}

uint16_t Receptor_Communication(void)
{
	uint16_t dist = distance;
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
		if(rx_buffer[0] == OurID)
		{
			uint32_t resp_tx_time;
			
			TargetID = rx_buffer[1];

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64(); //读取rx_deviceB_first_msg时间戳

			/* Set send time for response. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_reply_msg时间戳
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
			
			//修改tx_reply_msg

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
				if(rx_buffer[0] == OurID && rx_buffer[1] == TargetID)
				{
					uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
					uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64_t tof_dtu;

					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64(); //读取tx_reply_msg时间戳
					final_rx_ts = get_rx_timestamp_u64(); //读取rx_final_msg时间戳

					/* Get timestamps embedded in the final message. */
					//poll_tx_ts
					//resp_rx_ts
					//final_tx_ts

					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. */
					poll_rx_ts_32 = (uint32_t)poll_rx_ts;
					resp_tx_ts_32 = (uint32_t)resp_tx_ts;
					final_rx_ts_32 = (uint32_t)final_rx_ts;
					Ra = (double)(resp_rx_ts - poll_tx_ts);
					Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
					Da = (double)(final_tx_ts - resp_rx_ts);
					Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					dist = (uint16_t)(distance * 100);
												
					/* Send distance to initiator */
					//修改tx_final_msg
					dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
					dwt_writetxfctrl(sizeof(tx_final_msg), 0);
					dwt_starttx(DWT_START_TX_IMMEDIATE);
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
    uint8_t i;
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
    uint8_t i;
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
static void final_msg_get_ts(const uint8_t *ts_field, uint64_t *ts)
{
    uint8_t i;
    *ts = 0;
    for (i = 0; i < TimeStampLength; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
