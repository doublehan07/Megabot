/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ranging_api.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
#define UUS_TO_DWT_TIME 65536 //1 uus = 512 / 499.2 us and 1 us = 499.2 * 128 dtu. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000
#define POLL_RX_TO_RESP_TX_DLY_UUS 150
#define SPEED_OF_LIGHT 299702547

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u32 status_reg;
static u32 frame_len;

//Time-stamps of frames transmission/reception, expressed in device time units.
//As they are 40-bit wide, we need to define a 64-bit int type to handle them.
//For Initiator
static uint64_t tx_DeviceA_1;
static uint64_t rx_DeviceA_2[4] = {0}; //之后节点数量是动态设置的，记得开大空间
static uint64_t tx_DeviceA_3;

//For Recptor
static uint64_t rx_DeviceB_1;
static uint64_t tx_DeviceB_2;
static uint64_t rx_DeviceB_3;

static double Treply1 = 0.0;
static double Treply2 = 0.0;
static double Tround1 = 0.0;
static double Tround2 = 0.0;
static double Tprop = 0.0;
static int64_t tof_dtu;
static double tof = 0.0;
double distance = 0.0;
uint16_t distance_cm = 0;

const u8 N = 4; //之后节点数量是动态设置的，记得加接口

/* Private function prototypes -----------------------------------------------*/
u8 Receptor_Waiting(u8 *rx_buffer, u8 *length, u8 *emergency_stop); //return If_FAIL_COMMUNICATION(0x00 - T, 0xFF - F)
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static uint64_t get_sys_timestamp_u64(void);
static void set_ts(u8 *addr, uint64_t ts);
static uint64_t get_ts(u8 *addr);

/*
	Reply msg
	Cmdtype | SenderID | TargetID | CRC | CRC
	0xF2 | SenderID | 0x00 | 0x0D | 0x0A
*/
static u8 rply_brd_msg[5] = {0xF2, MyID, 0x00, 0x0D, 0x0A};

/*
	Final msg
	CmdType | SenderID | TimeStamp[4:0] * N| CRC | CRC
	0xF3 | 0x01 | ts[4:0] - N | 0x0D | 0x0A
*/
static u8 fnl_brd_msg[34] = {0xF3, MyID, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0x0D, 0x0A};

/* Private functions ---------------------------------------------------------*/
u8 SendMsg(u8 *pointer, u8 arrayLenth) //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
{
	u8 IF_FAIL_SENDING = 0xFF;
	
	dwt_writetxdata(arrayLenth, pointer, 0);
	dwt_writetxfctrl(arrayLenth, 0);
	if(dwt_starttx(DWT_START_TX_IMMEDIATE) == 0)
		IF_FAIL_SENDING = 0;
	
	//Try resend
	if(IF_FAIL_SENDING)
	{
		dwt_forcetrxoff(); 	
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);	
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		if(dwt_starttx(DWT_START_TX_IMMEDIATE) == 0)
			IF_FAIL_SENDING = 0;
	}
	
	return IF_FAIL_SENDING;
}

u8 Receptor_Listening(u8 *rx_buffer, u8 *length, u8 *emergency_stop, u16 timeout) //return If_FAIL_COMMUNICATION(0x00 - T, 0xFF - F)
{
	u8 If_FAIL_COMMUNICATION = 0xFF;
	
	dwt_forcetrxoff(); //Force IC back to IDLE mode.
	dwt_setrxtimeout(timeout); //Clear reception timeout to start next ranging process.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS); //Clear all events flag.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.

	dwt_rxenable(0); //Activate reception immediately.
	If_FAIL_COMMUNICATION = Receptor_Waiting(rx_buffer, length, emergency_stop);
	
	return If_FAIL_COMMUNICATION;
}

u8 Receptor_Waiting(u8 *rx_buffer, u8 *length, u8 *emergency_stop) //return If_FAIL_COMMUNICATION(0x00 - T, 0xFF - F)
{
	u8 If_FAIL_COMMUNICATION = 0xFF;
	
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) 
		if(*emergency_stop != 0)//Eeception of a frame or error/timeout.
			return 0x0F; //Been broken by upper layer.
	
	if (status_reg & SYS_STATUS_RXFCG)
	{
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); //Clear good RX frame event in the DW1000 status register.

		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023; //A frame has been received, read it into the local buffer.
		if (frame_len <= BUFFER_LENGTH)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
			*length = (u8)frame_len;
			If_FAIL_COMMUNICATION = 0;
		}
		else //overstack
			return 0x0A; //Vector out of range.
	}
	else
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.
	
	return If_FAIL_COMMUNICATION;
}

u8 Initiator_Ranging(u8 *data, u8 length) //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
{
/*CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
	    		| CoodinateY | Dist1(RESV) | Dist2(RESV) | CommCount | (CRC) | (CRC)
	0x0A | 0x01 | 0x02 | 0x04 | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 | (0x0D) | (0x0A)
*/
	u8 i;
	u8 ts_counter = 0;
	u8 Times = data[0];
	u8 RecieveIDLowerBound = data[2];
	u8 RecieveIDUpperBound = data[3];
	static u8 IF_FAIL_SENDING = 0xFF;
	static u8 rx_buffer[BUFFER_LENGTH] = {0};
	static u8 rx_length = 0;
	u32 final_tx_time;
	uint64_t sysTime, calcTime;
	
	//Initiator begin broadcasting to active a new localization process.
	dwt_forcetrxoff(); //Force IC back to IDLE mode.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS); //Clear all events flag.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.
	
	data[12] = Times; //Whether we need one more reply or not after finishing broadcast ranging.
	data[0] = 0xF1;
	
	dwt_setrxtimeout(5000); //Delay maximun ~5ms for each nodes. 
	
	dwt_writetxdata(length, data, 0);
	dwt_writetxfctrl(length, 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	
	tx_DeviceA_1 = get_tx_timestamp_u64();
	
	for(i = 0; i < N - 1; i++) //Receive N noodes' info
	{
		if(Receptor_Waiting(rx_buffer, &rx_length, 0) == 0) //Successfully receive data.
		{
			//Check that the frame is the expected response from Target Device(LBD <= TargetID < UBD).
			if(rx_buffer[0] == 0xF2 && rx_buffer[2] == MyID \
				&& rx_buffer[1] >= RecieveIDLowerBound && rx_buffer[1] < RecieveIDUpperBound)
			{
				//My data. Record the timestamp.
				rx_DeviceA_2[ts_counter++] = get_rx_timestamp_u64();
			}
		}
	}
	
	if(ts_counter == N - 1)
		IF_FAIL_SENDING = 0;
	
	sysTime = get_sys_timestamp_u64();
	//Compute final message transmission time.
	final_tx_time = (sysTime + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_second_range_msg时间戳
	dwt_setdelayedtrxtime(final_tx_time);
	tx_DeviceA_3 = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 
	
	//Final msg.
	for(i = 0; i < ts_counter; i++)
	{
		calcTime = rx_DeviceA_2[i] - tx_DeviceA_1;
		set_ts(fnl_brd_msg + 2, calcTime);
		calcTime = tx_DeviceA_3 - rx_DeviceA_2[i];
		set_ts(fnl_brd_msg + 7, calcTime);
	}
	dwt_writetxdata(sizeof(fnl_brd_msg), fnl_brd_msg, 0);
	dwt_writetxfctrl(sizeof(fnl_brd_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
	
	return IF_FAIL_SENDING;
}

u16 Receptor_Ranging(u8 delay, u8 leftnumber) //return IF_FAIL_GET_DIST(distance_cm, 0xFFFF - F)
{
	u8 RecieveIDLowerBound;
	u8 RecieveIDUpperBound;
	u8 SourceID;
	static u8 rx_buffer[BUFFER_LENGTH] = {0};
	static u8 rx_length = 0;
	u32 resp_tx_time;
	uint64_t sysTime;
	
	if(Receptor_Listening(rx_buffer, &rx_length, 0, 0) == 0) //Successfully receive data.
	{
		SourceID = rx_buffer[1];
		RecieveIDLowerBound = rx_buffer[2];
		RecieveIDUpperBound = rx_buffer[3];
		
		//Check that the frame is the expected response from Target Device(LBD <= TargetID < UBD).
		if(rx_buffer[0] == 0xF1 && MyID >= RecieveIDLowerBound && MyID < RecieveIDUpperBound)
		{
			rx_DeviceB_1 = get_rx_timestamp_u64(); //Retrieve poll reception timestamp.
			//I need reply when delay action is done.
			Delay(3 * delay);
			sysTime = get_sys_timestamp_u64();
			resp_tx_time = (sysTime + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8; //计算tx_reply_msg时间戳
			dwt_setdelayedtrxtime(resp_tx_time);
			
			rply_brd_msg[2] = SourceID;
			
			dwt_writetxdata(sizeof(rply_brd_msg), rply_brd_msg, 0);
			dwt_writetxfctrl(sizeof(rply_brd_msg), 0);
			dwt_starttx(DWT_START_TX_DELAYED);
			
			Delay(leftnumber * 3000);
			tx_DeviceB_2 = get_tx_timestamp_u64();
			
			if(Receptor_Listening(rx_buffer, &rx_length, 0, 5700) == 0)
			{
				if(rx_buffer[0] == 0xF3 && rx_buffer[1] == SourceID)
				{
					rx_DeviceB_3 = get_rx_timestamp_u64();
					Tround1 = (double)get_ts(rx_buffer + 2 + delay * 10);
					Treply2 = (double)get_ts(rx_buffer + 2 + 5 + delay * 10);
					Treply1 = (double)(tx_DeviceB_2 - rx_DeviceB_1);
					Tround2 = (double)(rx_DeviceB_3 - tx_DeviceB_2);
					
					Tprop = (Treply1 * Treply2 - Tround2 * Tround1) / (Treply1 + Treply2 + Tround1 + Tround2);
					Tprop = Tprop > 0 ? Tprop : -Tprop;
					
					tof_dtu = (int64_t)Tprop;
					tof = tof_dtu * DWT_TIME_UNITS;
					distance = tof * SPEED_OF_LIGHT;
					
					if(distance <= 2.0)
					{
						distance_cm = (uint16_t)(distance * 100);					
						return distance_cm;
					}
				}
			}
		}
	}
	return 0xFFFF;
}

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

static uint64_t get_sys_timestamp_u64(void)
{
	u8 ts_tab[5] = {0};
	uint64_t ts = 0;
	int8_t i;
	dwt_readsystime(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static void set_ts(u8 *addr, uint64_t ts)
{
	u8 i, temp;
	for(i = 0; i < 5; i++)
	{
		temp = (u8)ts;
		*(addr + i) = temp;
		ts >>= 8;
	}
}

static uint64_t get_ts(u8 *addr)
{
	uint64_t ts = 0;
	u8 i;
	for(i = 0; i < 5; i++)
	{
		ts |= *(addr + 4 - i);
		ts <<= 8;
	}
	return ts;
}
