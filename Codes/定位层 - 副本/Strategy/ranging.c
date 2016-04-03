/**
  ******************************************************************************
  * DW测距接口
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
static uint8_t tx_request_begin_range_msg[] = {0x01, 0x00, MyID, 0x01, 0, 0, 0, 0x0D, 0x0A};
static uint8_t tx_second_range_msg[] = {0x01, 0x00, MyID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};
static uint8_t tx_reply_msg[] = {0x01, 0x00, MyID, 0x01, 0x0D, 0x0A};
static uint8_t tx_final_msg[] = {0x01, 0x00, MyID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};
static uint8_t tx_request_second_corp_msg[] = {0x01, 0x00, MyID, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 0x0A};

//Double buffer listening
#define MSG_RXBUF_LENGTH	10
static u8 msg_Rx_Buffer[MSG_RXBUF_LENGTH];
static u8 msg_Rx_Buffer_dbl[MSG_RXBUF_LENGTH];

/* Buffer to store received response message. */
#define TS1_FIELD_INDEX 4
#define TS2_FIELD_INDEX 8
#define TS3_FIELD_INDEX 12
uint8 rx_buffer[RX_BUF_LEN];

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
u8 Initiator_Ranging(uint8_t TargetID, u8 Times, u8 MyStatus, u8 which_freq)
{	
	u8 flag = RESET;
	
	/* Force IC back to IDLE mode. */
	dwt_forcetrxoff(); //如果不让DW回到IDLE状态，设置timeout会失效，之后如不能接受会卡死在while循环里
	
	/* Clear All events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	/* Write frame data to DW1000 and prepare transmission. */	
	tx_request_begin_range_msg[1] = TargetID;
	tx_request_begin_range_msg[4] = Times;
	tx_request_begin_range_msg[5] = MyStatus;
	tx_request_begin_range_msg[6] = which_freq;
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
			tx_DeviceA_1 = get_tx_timestamp_u64(); //读取tx_request_begin_range_msg时间戳
			rx_DeviceA_2 = get_rx_timestamp_u64(); //读取rx_first_response_range_msg时间戳

			/* Compute final message transmission time. */
			final_tx_time = (rx_DeviceA_2 + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_second_range_msg时间戳
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			//把Tround1和Treply2告诉Target Device(Treply2加上antenna delay)
			tx_DeviceA_3 = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 
			
			//修改tx_second_range_msg
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
					/* 解析本次数据包中的tof_dtu */
					int64_t tof_dtu = 0;
					for(i = 0; i < 8; i++)
					{
						tof_dtu += rx_buffer[TS1_FIELD_INDEX + i] << (i * 8);
					}

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					distance_cm = (uint16_t)(distance * 100);
					
					flag = 0xAA;
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

u8 Receptor_Ranging(u8 *TargetID, u8 *Times, u8 *UpperStatus, u8 *which_freq)
{
	u8 flag = 0;
	u32 frame_len, resp_tx_time;
					
	*TargetID = rx_buffer[2];
	*Times = rx_buffer[4];
	*UpperStatus = rx_buffer[5];
	*which_freq = rx_buffer[6];

	/* Retrieve poll reception timestamp. */
	rx_DeviceB_1 = get_rx_timestamp_u64(); //读取rx_deviceB_first_msg时间戳

	/* Set send time for response. */
	resp_tx_time = (rx_DeviceB_1 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_reply_msg时间戳
	dwt_setdelayedtrxtime(resp_tx_time);

	/* Set expected delay and timeout for final message reception. */
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(3700);
			
	//修改tx_reply_msg
	tx_reply_msg[1] = *TargetID;

	/* Write and send the response message. */
	dwt_writetxdata(sizeof(tx_reply_msg), tx_reply_msg, 0);
	dwt_writetxfctrl(sizeof(tx_reply_msg), 0);
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

		/* Check that the frame is the expected message from Source Device(TargetID). */
		if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[2] == *TargetID && rx_buffer[3] == 0x02)
		{
			uint8_t i;
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
					
			flag = 0xAA; //flag = 0xAA, 测距成功
												
			/* Send distance to Source Device */
			//修改tx_final_msg
			for(i = 0; i < 8; i++)
			{
				tx_final_msg[TS1_FIELD_INDEX + i] = (uint8_t)tof_dtu;
				tof_dtu >>= 8;
			}
			tx_final_msg[1] = *TargetID;
					
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
	
	return flag;
}

u8 Corp_Ranging(uint8_t TargetID, u8 Times, u8 MyStatus, u8 which_freq, u16 *Axis)
{	
	u8 flag = RESET;
	
	/* Force IC back to IDLE mode. */
	dwt_forcetrxoff(); //如果不让DW回到IDLE状态，设置timeout会失效，之后如不能接受会卡死在while循环里
	
	/* Clear All events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	/* Write frame data to DW1000 and prepare transmission. */	
	tx_request_begin_range_msg[1] = TargetID;
	tx_request_begin_range_msg[4] = Times;
	tx_request_begin_range_msg[5] = MyStatus;
	tx_request_begin_range_msg[6] = which_freq;
	tx_request_second_corp_msg[16] = (u8)Axis[0];
	tx_request_second_corp_msg[17] = (u8)(Axis[0] >> 8);
	tx_request_second_corp_msg[18] = (u8)Axis[1];
	tx_request_second_corp_msg[19] = (u8)(Axis[1] >> 8);
	tx_request_second_corp_msg[20] = (u8)Axis[2];
	tx_request_second_corp_msg[21] = (u8)(Axis[2] >> 8);
	tx_request_second_corp_msg[22] = (u8)Axis[3];
	tx_request_second_corp_msg[23] = (u8)(Axis[3] >> 8);
	
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
			tx_DeviceA_1 = get_tx_timestamp_u64(); //读取tx_request_begin_range_msg时间戳
			rx_DeviceA_2 = get_rx_timestamp_u64(); //读取rx_first_response_range_msg时间戳

			/* Compute final message transmission time. */
			final_tx_time = (rx_DeviceA_2 + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_second_range_msg时间戳
			dwt_setdelayedtrxtime(final_tx_time);

			/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
			//把Tround1和Treply2告诉Target Device(Treply2加上antenna delay)
			tx_DeviceA_3 = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 
			
			//修改tx_second_range_msg
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
			dwt_writetxdata(sizeof(tx_request_second_corp_msg), tx_request_second_corp_msg, 0);
			dwt_writetxfctrl(sizeof(tx_request_second_corp_msg), 0);
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
					/* 解析本次数据包中的tof_dtu */
					int64_t tof_dtu = 0;
					for(i = 0; i < 8; i++)
					{
						tof_dtu += rx_buffer[TS1_FIELD_INDEX + i] << (i * 8);
					}

					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					distance_cm = (uint16_t)(distance * 100);
					
					flag = 0xAA;
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
	u8 flag = 0;
	
	dwt_forcetrxoff(); //Force IC back to IDLE mode.	
	dwt_setdblrxbuffmode(0); //Close double buffer mode.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event	
	dwt_setrxtimeout(0); //阻塞式监听

	dwt_rxenable(0); //Activate reception immediately.

	/* Eeception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{ 
		//这里要有监听被打断的机制吗？
		if(Usart_RX_flag == SET) //当上位机发来信息
		{
			return 0xEE;
		}
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
			flag = 1;
		}
	}
	else
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.
	}	
	return flag;
}

void Double_Buff_Recp_Listening(u8 *ID, u8 if_brd_use)
{
	static u8 counter = 0;
	static u32 status_reg, frame_len;
	
	//不要开双收模式的自动重启接收！
	//双收模式overrun的case太难处理了！
	if(if_brd_use == 0)
	{
		/*
		dwt_forcetrxoff(); //Force IC back to IDLE mode.	
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event	
		
		dwt_setautorxreenable(0); //禁止自动重启接收
		dwt_setdblrxbuffmode(1); //开启双收模式
			
		//要求HSRBP == ICRBP
		dwt_syncrxbufptrs();
		dwt_setrxtimeout(3000); //这个参数要调，一次接收的timeout时间
			
		dwt_rxenable(0); //Start RX
		*/
		dwt_forcetrxoff(); //Force IC back to IDLE mode.	
		dwt_setdblrxbuffmode(0); //Close double buffer mode.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event	
		dwt_setrxtimeout(0); //阻塞式监听

		dwt_rxenable(0); //Activate reception immediately.
	}
	
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	
	{
		if(Usart_RX_flag == SET) //当上位机发来信息
		{
			return;
		}
	}
		
	if(status_reg & SYS_STATUS_RXFCG)
	{		
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= MSG_RXBUF_LENGTH)
		{		
			dwt_readrxdata(msg_Rx_Buffer, frame_len, 0);
		}
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
//		dwt_syncrxbufptrs(); //切换保证两个相等
		dwt_setrxtimeout(5000); 
		counter++;
		dwt_rxenable(0); //马上开始接收下一个
			
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	{}
				
		if(status_reg & SYS_STATUS_RXFCG)
		{
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= MSG_RXBUF_LENGTH)
			{		
				dwt_readrxdata(msg_Rx_Buffer_dbl, frame_len, 0);
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD); //clear good&bad event
			dwt_forcetrxoff();
			counter++;
		}
		else
		{
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
		}
	}
	else
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	}
		
	ID[0] = 0;
	if(counter > 0)
	{
		//CmdType | MyID | ReserveForCounter | 0x0D | 0x0A
		if(msg_Rx_Buffer[0] == 0x07)
		{
			ID[1] = msg_Rx_Buffer[1];
			ID[0] = 1;
		}
		if(counter > 1)
		{
			if(msg_Rx_Buffer_dbl[0] == 0x07)
			{
				ID[ID[0]+1] = msg_Rx_Buffer_dbl[1];
				ID[0]++;
			}
		}
	}
	counter = 0;
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
