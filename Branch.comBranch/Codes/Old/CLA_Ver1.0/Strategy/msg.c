/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "msg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Broadcast_Msg(u8 CorpID, u8 if_bdc_axis)
{
	//CmdType | CorpID | MyID | RectXL | RectXH | RectYL | RectYH | 0x0D | 0x0A
	static u8 bdc_msg[9] = {0x03, 0, MyID, 0, 0, 0, 0, 0x0D, 0x0A};
	u16 RectX = (u16)myInfo.Rect_Axis[0];
	u16 RectY = (u16)myInfo.Rect_Axis[1];
	
	bdc_msg[0] = if_bdc_axis ? 0x05 : 0x03;
	bdc_msg[1] = CorpID;
	bdc_msg[3] = (u8)RectX;
	bdc_msg[4] = (u8)(RectX >> 8);
	bdc_msg[5] = (u8)RectY;
	bdc_msg[6] = (u8)(RectY >> 8);
	
	/* Start transmission */
	dwt_writetxdata(sizeof(bdc_msg), bdc_msg, 0);
	dwt_writetxfctrl(sizeof(bdc_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
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
	//CmdType | MyID | ReserveForCounter | 0x0D | 0x0A
	static u8 resp_msg[5] = {0x07, MyID, 0, 0x0D, 0x0A};
	
		/* Start transmission */
	dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
	dwt_writetxfctrl(sizeof(resp_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}
