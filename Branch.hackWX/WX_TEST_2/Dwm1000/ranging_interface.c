///**
//  ******************************************************************************
//  * 
//  ******************************************************************************
//  */

///* Includes ------------------------------------------------------------------*/
//#include "ranging_api.h"
//#include "deca_device_api.h"
//#include "deca_regs.h"

//#include <string.h>

///* Private typedef -----------------------------------------------------------*/
///* Private define ------------------------------------------------------------*/
////UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
//#define UUS_TO_DWT_TIME 65536 //1 uus = 512 / 499.2 us and 1 us = 499.2 * 128 dtu. */
////#define RESP_RX_TO_FINAL_TX_DLY_UUS 1000
////#define POLL_RX_TO_RESP_TX_DLY_UUS 1000
////#define SPEED_OF_LIGHT 299702547

///* Private macro -------------------------------------------------------------*/
///* Private variables ---------------------------------------------------------*/
//static u32 status_reg;

////Time-stamps of frames transmission/reception, expressed in device time units.
////As they are 40-bit wide, we need to define a 64-bit int type to handle them.
////For Initiator
//u64 tx_DeviceA_1;
//u64 rx_DeviceA_2a;
//u64 rx_DeviceA_2b;
//u64 rx_DeviceA_2c;
//u64 tx_DeviceA_3;

////For Recptor
//u64 rx_DeviceB_1;
//u64 tx_DeviceB_2;
//u64 rx_DeviceB_3;

//u32 frame_len;
//u8 rx_buffer[RX_BUFFER_LENGTH];

///* Private function prototypes -----------------------------------------------*/
////static void set_ts(u8 *addr, uint64_t ts);
////static u64 get_ts(u8 *addr);

///* Private functions ---------------------------------------------------------*/
///*
//	新的广播测距通信方式
//	1号点广播poll消息，发起测距请求，记录发送时间戳，之后立即转入半阻塞接收模式
//	//半阻塞接收:
//		调用函数设置tick的延时值，然后在接收的阻塞while循环里读tick值。当tick值为0时跳出循环终止阻塞
//		如果半阻塞的delay值在100ms以内，我觉得是可以接受的
//	2、3、4号点回复resp消息，//2、3、4读取自己接收和发送的时间戳，在消息里告诉1号点。
//	1号点读取接收到的一堆时间戳，同时计数接收的resp消息数量
//	（当数量达到或者timeout时，进行final消息）
//	于是1号点最终1对1带时间戳连续回复了2、3、4
//	
//	//在这个流程中，其实1是能估计距离的。。
//	准确的值是2去计算
//	
//	另外是可以压缩时间戳信息的 x.xxx可以用两个8字节安全传输 x.x xx
//	new dist-time unit 4.69e-3 两个8字节，这样压缩的话final放在一条消息里面吧233
//*/

//u8 Brd_Msg(void)
//{
//	/*	CmdType | SenderID | dist1a | dist1b | dist2a | dist2b | dist3a | dist3b | CRC | CRC
//			0xB0
//	*/
//	static u8 IF_FAIL_SENDING = 0xFF;
//	static u8 brdMsg[10] = {0};
//	
//	brdMsg[0] = 0xB0;
//	brdMsg[1] = MyID;
//	
//	brdMsg[2] = (u8)ourNodes[MyID].Dist[0]; //L
//	brdMsg[3] = (u8)(ourNodes[MyID].Dist[0]>>8); //H
//	
//	brdMsg[4] = (u8)ourNodes[MyID].Dist[1]; //L
//	brdMsg[5] = (u8)(ourNodes[MyID].Dist[1]>>8); //H
//	
//	brdMsg[6] = (u8)ourNodes[MyID].Dist[2]; //L
//	brdMsg[7] = (u8)(ourNodes[MyID].Dist[2]>>8); //H
//	
//	//CRC
//	//CRC
//	
//	IF_FAIL_SENDING = SendMsg(brdMsg, 10, 0);
//	
//	return IF_FAIL_SENDING;
//}
//u8 Poll_Msg(u8 LBD, u8 UBD, u16 MyX, u16 MyY)
//{
//	/*	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
//							| CoodinateY | CRC | CRC
//			LBD <= TargetID < UBD
//			0xF1
//	*/
//	static u8 IF_FAIL_SENDING = 0xFF;
//	static u8 pollMsg[10] = {0};
//	
//	pollMsg[0] = 0xF1; //CmdType
//	pollMsg[1] = MyID; //SenderID
//	pollMsg[2] = LBD; //ReceiveIDLowerBound
//	pollMsg[3] = UBD; //ReceiveIDUpperBound
//	pollMsg[4] = (u8)MyX; //xL
//	pollMsg[5] = (u8)(MyX>>8); //xH
//	pollMsg[6] = (u8)MyY; //yL
//	pollMsg[7] = (u8)(MyY>>8); //yH 
//	
//	//CRC
//	//CRC
//	
//	IF_FAIL_SENDING = SendMsg(pollMsg, 10, 0);
//	
//	if(!IF_FAIL_SENDING)
//	{
//		tx_DeviceA_1 = get_tx_timestamp_u64();
//	}
//	
//	return IF_FAIL_SENDING;
//}

//u8 Resp_Msg(u8 Target)
//{
//	/*	Cmdtype | SenderID | TargetID | CRC | CRC
//			0xF2
//	*/
//	static u8 IF_FAIL_SENDING = 0xFF;
//	static u8 respMsg[5] = {0};
//	
//	respMsg[0] = 0xF2; //CmdType
//	respMsg[1] = MyID; //SenderID
//	respMsg[2] = Target; //TargetID
//	
//	//CRC
//	//CRC
//	
//	IF_FAIL_SENDING = SendMsg(respMsg, 5, 0);
//	
//	if(!IF_FAIL_SENDING)
//	{
//		tx_DeviceB_2 = get_tx_timestamp_u64();
//	}
//	
//	return IF_FAIL_SENDING;
//}

//u8 Final_Msg(u16 delayTime, u64 TSa, u64 TSb1, u64 TSb2, u64 TSb3)
//{
//	/*	CmdType | SenderID | TimeStamp[4:0] * N| CRC | CRC
//			0xF3
//	*/
//	static u8 IF_FAIL_SENDING = 0xFF;
//	static u8 finalMsg[16] = {0};
//	static u32 final_tx_time;
//	static u64 temp_TS;
//	static u64 TSc;
//	static double temp_DIS;
//	
//	final_tx_time = (get_rx_timestamp_u64() + (delayTime * UUS_TO_DWT_TIME)) >> 8;
//	TSc = (((uint64_t)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY; 	
//	tx_DeviceA_3 = TSc;
//	
//	finalMsg[0] = 0xF3; //CmdType
//	finalMsg[1] = MyID; //SenderID
//	
//	temp_TS = TSb1 - TSa;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[2] = (u8)(temp_TS / 100.0); //1a
//	finalMsg[3] = (u8)(temp_TS - finalMsg[2] * 100.0); //1b
//	
//	temp_TS = TSc - TSb1;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[4] = (u8)(temp_TS / 100.0); //1c
//	finalMsg[5] = (u8)(temp_TS - finalMsg[4] * 100.0); //1d
//	
//	temp_TS = TSb2 - TSa;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[6] = (u8)(temp_TS / 100.0); //2a
//	finalMsg[7] = (u8)(temp_TS - finalMsg[6] * 100.0); //2b
//	
//	temp_TS = TSc - TSb2;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[8] = (u8)(temp_TS / 100.0); //2c
//	finalMsg[9] = (u8)(temp_TS - finalMsg[8] * 100.0); //2d
//	
//	temp_TS = TSb3 - TSa;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[10] = (u8)(temp_TS / 100.0); //3a
//	finalMsg[11] = (u8)(temp_TS - finalMsg[10] * 100.0); //3b
//	
//	temp_TS = TSc - TSb3;
//	temp_DIS = temp_TS * TIM_DIS_UNIT;
//	temp_TS = (int)(temp_DIS * 1000.0 + 0.5);
//	finalMsg[12] = (u8)(temp_TS / 100.0); //3c
//	finalMsg[13] = (u8)(temp_TS - finalMsg[12] * 100.0); //3d
//	
//	//CRC
//	//CRC
//	
//	IF_FAIL_SENDING = SendMsg(finalMsg, 16, final_tx_time);
//	
//	return IF_FAIL_SENDING;
//}

//u8 SendMsg(u8 *buffer, u8 arrayLenth, u32 final_tx_time) //return IF_FAIL_SENDING(0x00 - T, 0xFF - F)
//{
//	u8 IF_FAIL_SENDING = 0xFF;
//	
//	dwt_forcetrxoff(); 	
//	
//	dwt_writetxdata(arrayLenth, buffer, 0);
//	dwt_writetxfctrl(arrayLenth, 0);
//	
//	if(final_tx_time != 0) //DWT_START_TX_DELAYED
//	{
//		dwt_setrxtimeout(final_tx_time); //Set timeout to start next ranging process.
//		if(dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS)
//		{
//			IF_FAIL_SENDING = 0;
//		}
//	}
//	else
//	{
//		if(dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
//		{
//			IF_FAIL_SENDING = 0;
//		}
//	}
//	
//	//Try resend
//	if(IF_FAIL_SENDING)
//	{
//		dwt_forcetrxoff(); 	
//		
//		dwt_writetxdata(arrayLenth, buffer, 0);
//		dwt_writetxfctrl(arrayLenth, 0);
//		
//		if(dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_SUCCESS)
//		{
//			IF_FAIL_SENDING = 0;
//		}
//	}
//	
//	return IF_FAIL_SENDING;
//}

//u8 Safe_Receive(u32 timeout)
//{
//	u8 If_FAIL_COMMUNICATION = 0xFF;
//	
//	dwt_forcetrxoff(); //Force IC back to IDLE mode.
//	dwt_setrxtimeout(0); //Set reception timeout to start next ranging process.
//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS); //Clear all events flag.
//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.

//	dwt_rxenable(0); //Activate reception immediately.
//	SetTick(timeout);
//	
//	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) 
//	{
//		if(GetTick() == 0)//Timeout.
//			return 0x0F; //Timeout
//	}
//	
//	if (status_reg & SYS_STATUS_RXFCG)
//	{
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG); //Clear good RX frame event in the DW1000 status register.

//		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023; //A frame has been received, read it into the local buffer.
//		if (frame_len <= RX_BUFFER_LENGTH)
//		{
//			dwt_readrxdata(rx_buffer, frame_len, 0);
//			If_FAIL_COMMUNICATION = 0;
//		}
//		else //overstack
//		{
//			return 0x0A; //Vector out of range.
//		}
//	}
//	else
//	{
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); //Clear RX error events in the DW1000 status register.
//	}
//	
//	return If_FAIL_COMMUNICATION;
//}

//u64 get_tx_timestamp_u64(void)
//{
//	uint8_t ts_tab[5];
//	uint64_t ts = 0;
//	int8_t i;
//	dwt_readtxtimestamp(ts_tab);
//	for (i = 4; i >= 0; i--)
//	{
//		ts <<= 8;
//		ts |= ts_tab[i];
//	}
//	return ts;
//}

//u64 get_rx_timestamp_u64(void)
//{
//	uint8_t ts_tab[5];
//	uint64_t ts = 0;
//	int8_t i;
//	dwt_readrxtimestamp(ts_tab);
//	for (i = 4; i >= 0; i--)
//	{
//		ts <<= 8;
//		ts |= ts_tab[i];
//	}
//	return ts;
//}

//u64 get_sys_timestamp_u64(void)
//{
//	u8 ts_tab[5] = {0};
//	uint64_t ts = 0;
//	int8_t i;
//	dwt_readsystime(ts_tab);
//	for (i = 4; i >= 0; i--)
//	{
//		ts <<= 8;
//		ts |= ts_tab[i];
//	}
//	return ts;
//}

////static void set_ts(u8 *addr, uint64_t ts)
////{
////	u8 i, temp;
////	for(i = 0; i < 5; i++)
////	{
////		temp = (u8)ts;
////		*(addr + i) = temp;
////		ts >>= 8;
////	}
////}

////static u64 get_ts(u8 *addr)
////{
////	uint64_t ts = 0;
////	u8 i;
////	for(i = 0; i < 5; i++)
////	{
////		ts |= *(addr + 4 - i);
////		ts <<= 8;
////	}
////	return ts;
////}
