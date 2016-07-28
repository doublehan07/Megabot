/* Includes ------------------------------------------------------------------*/
#include "receptor.h"
#include "deca_regs.h"
#include "deca_device_api.h"

#include <string.h>

/* Frames used in the ranging process. See NOTE 2 below. */
static u8 rx_poll_msg[] = {0xF1, MyID, 0, 0x00, 0x00};
static u8 tx_resp_msg[] = {0xF2, MyID, 0, 0x00, 0x00};
static u8 rx_final_msg[] = {0xF3, MyID, 0, 0x00, 0, 0, 0, 0x00, 0, 0, 0, 0x00, 0, 0, 0, 0x00, 0x00};
//static u8 tx_exd_msg[] = {0xF4, MyID, 0, 0, 0, 0x00, 0x00};
static u8 cmp_msg[] = {0, 0, MyID};

#define FINAL_MSG_POLL_TX_TS_IDX 3
#define FINAL_MSG_RESP_RX_TS_IDX 7
#define FINAL_MSG_FINAL_TX_TS_IDX 11
#define FINAL_MSG_TS_LEN 4

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static u8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static u32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;
static u16 dist_cm;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const u8 *ts_field, u32 *ts);

static uint64 get_tx_timestamp_u64(void)
{
    u8 ts_tab[5];
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

static uint64 get_rx_timestamp_u64(void)
{
    u8 ts_tab[5];
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

static void final_msg_get_ts(const u8 *ts_field, u32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}


void Receptor_Ranging(void)
{
	static u8 TargetID = 0x00;
	
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);
    
	/* Activate reception immediately. */
	dwt_rxenable(0);
   
	/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ };
    
	if (status_reg & SYS_STATUS_RXFCG)
	{
		u32 frame_len;
      
		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
      
		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}
      
		if (rx_buffer[0] == 0xF1 && rx_buffer[2] == MyID)
		{
			u32 resp_tx_time;
			 
			TargetID = rx_buffer[1];
			tx_resp_msg[2] = TargetID;
//			tx_exd_msg[2] = TargetID;
        
			/* Retrieve poll reception timestamp. */
			poll_rx_ts = get_rx_timestamp_u64();
        
			/* Set send time for response. See NOTE 8 below. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);
        
			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
        
			/* Write and send the response message. See NOTE 9 below.*/
			dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
			dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
        
			/* We assume that the transmission is achieved correctly, now poll for reception of expected "final" frame or error/timeout.
			* See NOTE 7 below. */
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
				
        cmp_msg[0] = 0xF3;
				cmp_msg[1] = TargetID;
				if (memcmp(rx_buffer, cmp_msg, 3) == 0)
				{
					u32 poll_tx_ts, resp_rx_ts, final_tx_ts;
					u32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
					double Ra, Rb, Da, Db;
					int64 tof_dtu;
            
					/* Retrieve response transmission and final reception timestamps. */
					resp_tx_ts = get_tx_timestamp_u64();
					final_rx_ts = get_rx_timestamp_u64();
            
					/* Get timestamps embedded in the final message. */
					final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
					final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
            
					/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
					poll_rx_ts_32 = (uint32)poll_rx_ts;
					resp_tx_ts_32 = (uint32)resp_tx_ts;
					final_rx_ts_32 = (uint32)final_rx_ts;
					Ra = (double)(resp_rx_ts - poll_tx_ts);
					Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
					Da = (double)(final_tx_ts - resp_rx_ts);
					Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
					tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
            
					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT; 
					
					dist_cm = (u16)(distance * 1000);
//					tx_exd_msg[3] = (u8)(dist_cm >> 8); //H
//					tx_exd_msg[4] = (u8)dist_cm;				//L
					
//					/* Write and send the response message. See NOTE 9 below.*/
//					dwt_writetxdata(sizeof(tx_exd_msg), tx_exd_msg, 0);
//					dwt_writetxfctrl(sizeof(tx_exd_msg), 0);
//					dwt_starttx(DWT_START_TX_IMMEDIATE);
//					
//					/* Poll DW1000 until TX frame sent event set. See NOTE 8 below. */
//					while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
//					{ };
//        
//					/* Clear TXFRS event. */
//					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);   
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
}
