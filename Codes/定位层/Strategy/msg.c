/**
  ******************************************************************************
  * 
		Deca通信协议
		type = 0x01 - 与指定id进行测距				id = 测距目标id					(若不busy，被指定必须回应开始测距)
		type = 0x03 - 广播消息，必须回应      id = 协作点id
		type = 0x04 - 指定消息								id = 被指定点id         (频段根据自己是协作点还是主动点身份自动选择)
		type = 0x05 - 广播自己坐标消息    
		type = 0x06 - 开始新一轮的定位，大boss
		type = 0x07 - 回应广播消息					
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "msg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MSG_RXBUF_LENGTH	10
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u8 msg_Rx_Buffer[MSG_RXBUF_LENGTH];
static u8 msg_Rx_Buffer_dbl[MSG_RXBUF_LENGTH];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Broadcast_Msg(u8 CorpID, u8 if_bdc_axis, u8 *ID)
{
	//CmdType | CorpID | MyID | RectXL | RectXH | RectYL | RectYH | 0x0D
	static u8 counter = 0;
	static u8 bdc_msg[8] = {0x03, 0, MyID, 0, 0, 0, 0, 0x0D};
	static u32 status_reg, frame_len;
	u16 RectX = (u16)myInfo.Rect_Axis[0];
	u16 RectY = (u16)myInfo.Rect_Axis[1];
	
	bdc_msg[0] = if_bdc_axis ? 0x05 : 0x03;
	bdc_msg[1] = CorpID;
	bdc_msg[3] = (u8)RectX;
	bdc_msg[4] = (u8)(RectX >> 8);
	bdc_msg[5] = (u8)RectY;
	bdc_msg[6] = (u8)(RectY >> 8);
	
	/* Start transmission */
	dwt_forcetrxoff(); //如果不让DW回到IDLE状态，设置timeout会失效，之后如不能接受会卡死在while循环里
	
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_writetxdata(sizeof(bdc_msg), bdc_msg, 0);
	dwt_writetxfctrl(sizeof(bdc_msg), 0);
	
	if(if_bdc_axis)
	{
		dwt_starttx(DWT_START_TX_IMMEDIATE);
	}
	else //广播消息是期待回应的，我要记录我听到的前两个点
	{
		//不要开双收模式的自动重启接收！
		//双收模式overrun的case太难处理了！
		
		dwt_setautorxreenable(0); //禁止自动重启接收
		dwt_setdblrxbuffmode(1); //开启双收模式
		
		//要求HSRBP == ICRBP
		dwt_syncrxbufptrs();
		
		dwt_setrxtimeout(0); //阻塞式等待
		
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
		
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))	{}
		
		if(status_reg & SYS_STATUS_RXFCG)
		{		
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= MSG_RXBUF_LENGTH)
			{		
				dwt_readrxdata(msg_Rx_Buffer, frame_len, 0);
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD); //clear good&bad event
			dwt_syncrxbufptrs(); //切换保证两个相等
			dwt_rxenable(0); //马上开始接收下一个
			counter++;
			
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
			//CmdType | MyID | ReserveForCounter | 0x0D
			if(msg_Rx_Buffer[0] == 0x07 && msg_Rx_Buffer[3] == 0x0D)
			{
				ID[1] = msg_Rx_Buffer[1];
				ID[0] = 1;
			}
			if(counter > 1)
			{
				if(msg_Rx_Buffer_dbl[0] == 0x07 && msg_Rx_Buffer_dbl[3] == 0x0D)
				{
					ID[ID[0]+1] = msg_Rx_Buffer_dbl[1];
					ID[0]++;
				}
			}
		}
		counter = 0;
	}
}

void Selected_Msg(u8 SelectedID, u8 Frec)
{
	//CmdType | SelectedID | MyID | Frequence | 0x0D | 0x0A
	static u8 slc_msg[6] = {0x04, 0, MyID, 0, 0x0D, 0x0A};
	
	slc_msg[1] = SelectedID;
	slc_msg[3] = Frec;
	
	/* Start transmission */
	dwt_writetxdata(sizeof(slc_msg), slc_msg, 0);
	dwt_writetxfctrl(sizeof(slc_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void Boss_Msg(void)
{
	//CmdType | 0x0D | 0x0A
	static u8 boss_msg[3] = {0x06, 0x0D, 0x0A};
	
	/* Start transmission */
	dwt_writetxdata(sizeof(boss_msg), boss_msg, 0);
	dwt_writetxfctrl(sizeof(boss_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void Resp_Msg(void)
{
	//CmdType | MyID | ReserveForCounter | 0x0D
	static u8 resp_msg[4] = {0x07, MyID, 0, 0x0D};
	
		/* Start transmission */
	dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
	dwt_writetxfctrl(sizeof(resp_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}
