/**
  ******************************************************************************
  * 说明爱写不写，反正这是主函数代码
	* TOA-发送端
  ******************************************************************************
  */
	
#include <string.h>
#include <math.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"

static uint16_t DISTANCE_REQUIRMENT = 30;

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_dis_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x27, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static double distance;
static uint16_t distance_cm;
static uint16_t dis_compare;

/* Declaration of static functions. */
uint16_t Initiator_Communication(void);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

int main(void)
{
		uint16_t abs_value;
	
    /* Start with board specific hardware init. */
    peripherals_init();

    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_DW1000_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_DW1000_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. 
		 */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	
		motor_setspeed(MOTOR_ALL, 0);
		motor_move(MOTOR_ALL, MOTOR_FORWARD);

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
			distance_cm = Initiator_Communication();
			//distance = distance_cm / 100.0; debug - purpose
			
			abs_value = distance_cm - DISTANCE_REQUIRMENT;
			abs_value = abs_value > 0 ? abs_value : -abs_value;
			
			if(abs_value > 10)
			{
				if(distance_cm < DISTANCE_REQUIRMENT)
				{
//					if(distance_cm > dis_compare) //keep going
//					{
//						if(get_motor_direction(MOTOR_LEFT)) //backward
//						{
//							motor_move(MOTOR_ALL, MOTOR_BACKWARD);
//						}	
//						else
//						{
							motor_move(MOTOR_ALL, MOTOR_BACKWARD);
//						}
						motor_setspeed(MOTOR_ALL, 100);
//					}
//					else //reverse state
//					{
//						if(get_motor_direction(MOTOR_LEFT)) //backward
//						{
//							motor_move(MOTOR_ALL, MOTOR_FORWARD);
//						}	
//						else
//						{
//							motor_move(MOTOR_ALL, MOTOR_BACKWARD);
//						}
//						motor_setspeed(MOTOR_ALL, 600);
//					}
				}
				else
				{
//					if(distance_cm < dis_compare) //keep going
//					{
//						if(get_motor_direction(MOTOR_LEFT)) //backward
//						{
//							motor_move(MOTOR_ALL, MOTOR_BACKWARD);
//						}	
//						else
//						{
							motor_move(MOTOR_ALL, MOTOR_FORWARD);
//						}
						motor_setspeed(MOTOR_ALL, 100);
//					}
//					else //reverse state
//					{
//						if(get_motor_direction(MOTOR_LEFT)) //backward
//						{
//							motor_move(MOTOR_ALL, MOTOR_FORWARD);
//						}	
//						else
//						{
//							motor_move(MOTOR_ALL, MOTOR_BACKWARD);
//						}
//						motor_setspeed(MOTOR_ALL, 600);
//					}
				}
			}
			else
			{
					//motor_setspeed(MOTOR_ALL, 300);
					//motor_move(MOTOR_ALL, MOTOR_FORWARD);
				motor_setspeed(MOTOR_ALL, 0);
			}
		}
}

uint16_t Initiator_Communication(void)
{
	uint16_t dist = 0;
	dis_compare = distance_cm;
	/* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
	//static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; //tx_poll_msg[2] = 0

	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	 * set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ };

	/* Increment frame sequence number after transmission of the poll message (modulo 256). */
	frame_seq_nb++;

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32 frame_len;

		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}

		/* Check that the frame is the expected response from the companion "DS TWR responder" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		rx_buffer[ALL_MSG_SN_IDX] = 0; //rx_buffer[2] = 0
		//static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
		if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
		{
			uint32 final_tx_time;

			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64();
			resp_rx_ts = get_rx_timestamp_u64();

			/* Compute final message transmission time. See NOTE 9 below. */
			final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 10 below. */
			//static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts); //tx_final_msg[10] = poll_tx_ts
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts); //tx_final_msg[14] = resp_rx_ts
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts); //tx_final_msg[18] = final_tx_ts

			/* Write and send final message. See NOTE 7 below. */
			tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb; //tx_final_msg[2] = frame_seq_nb
			dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
			dwt_writetxfctrl(sizeof(tx_final_msg), 0);
			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
								
			/* We assume that the transmission is achieved correctly, now poll for reception of expected "final" frame or error/timeout. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
			{ };
								
			/* Increment frame sequence number after transmission of the final message (modulo 256). */
			frame_seq_nb++;
								
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

				/* Check that the frame is the expected response from the companion "DS TWR responder" example.
				 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
				rx_buffer[ALL_MSG_SN_IDX] = 0; //rx_buffer[2] = 0
				//static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
				//static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
				if (memcmp(rx_buffer, rx_dis_msg, ALL_MSG_COMMON_LEN) == 0)
				{
					//get distance
					dist = (rx_buffer[10] << 8) | rx_buffer[11];		
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
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
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
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
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
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
