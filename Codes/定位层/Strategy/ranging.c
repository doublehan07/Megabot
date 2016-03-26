/**
  ******************************************************************************
  * DW���ӿ�
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ranging.h"

/* Constant definition for calculation ---------------------------------------*/

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 us and 1 us = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function.
 * This includes the frame length of approximately ??? ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2500

/* Delay between frames, in UWB microseconds. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. 
 * This includes the frame length of approximately ??? ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Ranging definition -------------------------------------------------------*/
//CmdType | Target ID | MyID | Times | TimeStamp | CRC | CRC
static uint8_t tx_request_begin_range_msg[] = {0x01, 0x00, MyID, 0x01, 0x0D, 0x0A};
static uint8_t tx_second_range_msg[] = {0x01, 0x00, MyID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};
static uint8_t tx_reply_msg[] = {0x01, 0x00, MyID, 0x01, 0x0D, 0x0A};
static uint8_t tx_final_msg[] = {0x01, 0x00, MyID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};

/* Buffer to store received response message. */
#define RX_BUF_LEN 30
#define TS1_FIELD_INDEX 4
#define TS2_FIELD_INDEX 8
#define TS3_FIELD_INDEX 12
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
static uint64_t tx_DeviceA_1;
static uint64_t rx_DeviceA_2;
static uint64_t tx_DeviceA_3;

static uint64_t rx_DeviceB_1;
static uint64_t tx_DeviceB_2;
static uint64_t rx_DeviceB_3;

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof = 0.0;
static double Treply1 = 0.0;
static double Treply2 = 0.0;
static double Tround1 = 0.0;
static double Tround2 = 0.0;
static double Tprop = 0.0;
double distance = 0.0;
uint16_t distance_cm = 0;

/* Private function prototypes -----------------------------------------------*/
u16 Ranging_Communication(u8 *TargetID, u32 *frame_len, u8 *flag);
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

/* Private functions ---------------------------------------------------------*/
u8 Initiator_Ranging(uint8_t TargetID)
{	
	u8 flag = RESET;
	
	/* Force IC back to IDLE mode. */
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	
	/* Clear All events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	/* Write frame data to DW1000 and prepare transmission. */	
	tx_request_begin_range_msg[1] = TargetID;
	tx_second_range_msg[1] = TargetID;
	
	/* Set expected delay and timeout for final message reception. */
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(2700);
	
	/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame 
	 * is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
	dwt_writetxdata(sizeof(tx_request_begin_range_msg), tx_request_begin_range_msg, 0);
	dwt_writetxfctrl(sizeof(tx_request_begin_range_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	/* We assume that the transmission is achieved correctly */
	//Transmisson Error Handler()
	
	/* Reception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {}

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
		if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[2] == TargetID && rx_buffer[3] == 0x01)
		{
			uint32_t final_tx_time, temp_send_msg;
			uint8_t i;

			/* Retrieve poll transmission and response reception timestamp. */
			tx_DeviceA_1 = get_tx_timestamp_u64(); //��ȡtx_request_begin_range_msgʱ���
			rx_DeviceA_2 = get_rx_timestamp_u64(); //��ȡrx_first_response_range_msgʱ���

			/* Compute final message transmission time. */
			final_tx_time = (rx_DeviceA_2 + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //����tx_second_range_msgʱ���
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			//��Tround1��Treply2����Target Device(Treply2����antenna delay)
			tx_DeviceA_3 = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 
			
			//�޸�tx_second_range_msg
			temp_send_msg = (uint32_t)tx_DeviceA_1;
			for(i = 0; i < 4; i++)
			{
				tx_second_range_msg[TS1_FIELD_INDEX + i] = (uint8_t)temp_send_msg;
				temp_send_msg >>= 8;
			}
			
			temp_send_msg = (uint32_t)rx_DeviceA_2;
			for(i = 0; i < 4; i++)
			{
				tx_second_range_msg[TS2_FIELD_INDEX + i] = (uint8_t)temp_send_msg;
				temp_send_msg >>= 8;
			}
			
			temp_send_msg = (uint32_t)tx_DeviceA_3;
			for(i = 0; i < 4; i++)
			{
				tx_second_range_msg[TS3_FIELD_INDEX + i] = (uint8_t)temp_send_msg;
				temp_send_msg >>= 8;
			}

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(5000);
			
			/* Write and send final message. */
			dwt_writetxdata(sizeof(tx_second_range_msg), tx_second_range_msg, 0);
			dwt_writetxfctrl(sizeof(tx_second_range_msg), 0);
			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
								
			/* We assume that the transmission is achieved correctly */
			//Transmisson Error Handler()
	
			/* Reception of a frame or error/timeout. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {}
															
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
				if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[2] == TargetID && rx_buffer[3] == 0x02)
				{
					//get distance
					/* �����������ݰ��е�tof_dtu */
					int64_t tof_dtu = 0;
					for(i = 0; i < 8; i++)
					{
						tof_dtu += rx_buffer[TS1_FIELD_INDEX + i] << (i * 8);
					}

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					distance_cm = (uint16_t)(distance * 100);
					
					flag = SET;
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
		
	return flag;
}

u8 Receptor_Listening(void)
{
	u8 flag = RESET;
	
	dwt_forcetrxoff(); //Force IC back to IDLE mode.	
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event	
	dwt_setrxtimeout(0); //����ʽ����
	dwt_rxenable(0); //Activate reception immediately.

	/* Eeception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ 
		//����Ҫ�м�������ϵĻ�����
	}

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32_t frame_len;
		
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); //Clear good RX frame event in the DW1000 status register.

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
			flag = SET;
		}
	}
	else
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.
	}	
	return flag;
}

static uint64_t get_tx_timestamp_u64(void) //Get the TX time-stamp in a 64-bit variable. Assumes that length of time-stamps is 40 bits.
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

static uint64_t get_rx_timestamp_u64(void) //Get the RX time-stamp in a 64-bit variable. Assumes that length of time-stamps is 40 bits.
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
