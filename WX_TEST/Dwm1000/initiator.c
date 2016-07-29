#include "initiator.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "ranging_api.h"

#include <string.h>

/* Frames used in the ranging process. See NOTE 2 below. */
static u8 tx_poll_msg[] = {0xF1, MyID, 0, 0, 0};
static u8 rx_resp_msg[] = {0xF2, MyID, 0, 0, 0};
static u8 tx_final_msg[] = {0xF3, MyID, 0, 0x00, 0, 0, 0, 0x00, 0, 0, 0, 0x00, 0x00};
static u8 rx_exd_msg[] = {0xF4, MyID, 0, 0, 0, 0x00, 0x00};
static u8 cmp_msg[] = {0, 0, MyID};

static u8 upper_data[] = {0x0D, 0, 0, 0, 0, 0x0A};

#define FINAL_MSG_FRT_DIFF_TS_IDX 3
#define FINAL_MSG_SND_DIFF_TS_IDX 7
#define FINAL_MSG_TS_LEN 4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static u8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static u32 status_reg = 0;
	
/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static u16 dist_cm;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(u8 *ts_field, uint64 ts);

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

static void final_msg_set_ts(u8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (u8) ts;
        ts >>= 8;
    }
}

u8 Initiator_Ranging(u8 TargetID)
{
	int dwt_ret;
	u8 IF_FAIL_SENDING = 0xFF;
		
	tx_poll_msg[2] = TargetID;
	tx_final_msg[2] = TargetID;
	
	dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
	dwt_writetxfctrl(sizeof(tx_poll_msg), 0);
    
	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
	* set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    
	/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ };
        
	if (status_reg & SYS_STATUS_RXFCG)
	{
		u32 frame_len;
      
		/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
      
		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}
		
    cmp_msg[0] = 0xF2;
		cmp_msg[1] = TargetID;
		if (memcmp(rx_buffer, cmp_msg, 3) == 0)
		{
			u8 i;
			u32 final_tx_time;
			uint64 diff_ts;
        
			/* Retrieve poll transmission and response reception timestamp. */
			poll_tx_ts = get_tx_timestamp_u64();
			resp_rx_ts = get_rx_timestamp_u64();
        
			/* Compute final message transmission time. See NOTE 9 below. */
			final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(final_tx_time);
        
			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;
        
			/* Write all timestamps in the final message. See NOTE 10 below. */
			diff_ts = resp_rx_ts - poll_tx_ts;
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_FRT_DIFF_TS_IDX], diff_ts);
			diff_ts = final_tx_ts - resp_rx_ts;
			final_msg_set_ts(&tx_final_msg[FINAL_MSG_SND_DIFF_TS_IDX], diff_ts);
      
			//Dont need to set time, it has been done from main.
			
			dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
			dwt_writetxfctrl(sizeof(tx_final_msg), 0);		
			dwt_starttx(DWT_START_TX_DELAYED);
			
			/* Poll DW1000 until TX frame sent event set. See NOTE 8 below. */
			while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
			{ };
        
			/* Clear TXFRS event. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);  
		
			for(i = 0; i < 6; i++)
			{
				USART_SendData(USART2, upper_data[i]);
				while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
			}
			
			IF_FAIL_SENDING = 0;
		
//			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
//      
//			/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
//			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
//			{ };
//        
//			if (status_reg & SYS_STATUS_RXFCG)
//			{
//				u32 frame_len;
//      
//				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
//				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
//      
//				/* A frame has been received, read it into the local buffer. */
//				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
//				if (frame_len <= RX_BUF_LEN)
//				{
//					dwt_readrxdata(rx_buffer, frame_len, 0);
//				}
//      
//				if (rx_buffer[0] == 0xF4 && rx_buffer[1] == TargetID && rx_buffer[2] == MyID)
//				{
//					dist_cm = (rx_exd_msg[3] << 8) | rx_exd_msg[4];
//				}
//			}


		}
	}
	else
	{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	}
    
	/* Execute a delay between ranging exchanges. */
//	Delay(RNG_DELAY_MS);
	return IF_FAIL_SENDING;
}
