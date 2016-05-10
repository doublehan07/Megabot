/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ranging_api.h"
#include "deca_device_api.h"
#include "deca_regs.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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

#define SPEED_OF_LIGHT 299702547 // Speed of light in air, in metres per second.

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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
static u32 status_reg = 0;

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
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

/* Private functions ---------------------------------------------------------*/
uint8_t Initiator_Communication(uint8_t TargetID)
{	
	uint8_t flag = RESET;
	
	dwt_setGPIOvalue(GDM3, 0);
	dwt_setGPIOvalue(GDM0, GDP0);
	
	/* Force IC back to IDLE mode. */
	dwt_forcetrxoff(); //如果不让DW回到IDLE状态，设置timeout会失效，之后如不能接受会卡死在while循环里
	
	/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
	
	/* Clear All error events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
	
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
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{	
	}

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
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
			{		
			}
															
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
					
					if(distance <= 2.0)
					{
						distance_cm = (uint16_t)(distance * 100);
						
						/* Debug purpose. Test for getting correct distance value speed. */
//					dwt_setGPIOvalue(GDM0, GDP0);
						flag = SET;
					}
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
		
	return flag;
}

uint16_t Receptor_Communication(void)
{
	uint8_t TargetID;
	uint8_t flag = RESET;
	uint32_t resp_tx_time;
	uint8_t i;
	
//	dwt_setGPIOvalue(GDM3, GDP3);
//	dwt_setGPIOvalue(GDM0, 0);
	
	/* Force IC back to IDLE mode. */
	dwt_forcetrxoff();
	
	/* Clear reception timeout to start next ranging process. */
	dwt_setrxtimeout(0);
	
	/* Clear RX error events in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

	/* Activate reception immediately. */
	dwt_rxenable(0);

	/* Eeception of a frame or error/timeout. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {}

	if (status_reg & SYS_STATUS_RXFCG)
	{
		uint32_t frame_len;
		
		dwt_setGPIOvalue(GDM1, GDP1);
		
		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		/* A frame has been received, read it into the local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= RX_BUFFER_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}	
		
		if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[3] == 0x01) //如果是对方的测距请求
		{
//			dwt_setGPIOvalue(GDM1, 0);
			
			TargetID = rx_buffer[2];

			/* Retrieve poll reception timestamp. */
			rx_DeviceB_1 = get_rx_timestamp_u64(); //读取rx_deviceB_first_msg时间戳

			/* Set send time for response. */
			resp_tx_time = (rx_DeviceB_1 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_reply_msg时间戳
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(0);
			dwt_setrxtimeout(3700);
			
			//修改tx_reply_msg
			tx_reply_msg[1] = TargetID;

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
				dwt_setGPIOvalue(GDM1, GDP1);
				
				/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}

				/* Check that the frame is the expected message from Source Device(TargetID). */
				if(rx_buffer[0] == 0x01 && rx_buffer[1] == MyID && rx_buffer[2] == TargetID && rx_buffer[3] == 0x02)
				{
					int64_t tof_dtu;

					dwt_setGPIOvalue(GDM1, 0);
					dwt_setGPIOvalue(GDM2, GDP2);
					
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
					
					if(distance <= 2.0)
					{
						distance_cm = (uint16_t)(distance * 100);
						
						flag = 0xAA; //flag = 0xAA, 测距成功
						
						/* Debug purpose. Test for getting correct distance value speed. */
//					dwt_setGPIOvalue(GDM0, GDP0);
													
						/* Send distance to Source Device */
						//修改tx_final_msg
						for(i = 0; i < 8; i++)
						{
							tx_final_msg[TS1_FIELD_INDEX + i] = (uint8_t)tof_dtu;
							tof_dtu >>= 8;
						}
						tx_final_msg[1] = TargetID;
						
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
						dwt_writetxfctrl(sizeof(tx_final_msg), 0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);
					}
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
	
	dwt_setGPIOvalue(GDM1, 0);
//	dwt_setGPIOvalue(GDM2, 0);
	
	return (uint16_t)((TargetID << 8) | flag);
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
 *·
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
