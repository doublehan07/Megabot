/**
  ******************************************************************************
  * 
		Decaͨ��Э��
		type = 0x01 - ��ָ��id���в��				id = ���Ŀ��id					(����busy����ָ�������Ӧ��ʼ���)
		type = 0x03 - �㲥��Ϣ�������Ӧ      id = Э����id
		type = 0x04 - ָ����Ϣ								id = ��ָ����id         (Ƶ�θ����Լ���Э���㻹������������Զ�ѡ��)
		type = 0x05 - �㲥�Լ�������Ϣ    
		type = 0x06 - ��ʼ��һ�ֵĶ�λ����boss
		type = 0x07 - ��Ӧ�㲥��Ϣ					
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
void Broadcast_Msg(u8 CorpID, u8 *ID)
{
	//CmdType | CorpID | MyID | RectXL | RectXH | RectYL | RectYH | CRC | CRC
	static u8 bdc_msg[9] = {0x03, 0, MyID, 0, 0, 0, 0, 0x0D, 0x0A};
	
	u16 RectX = (u16)myInfo.Rect_Axis[0];
	u16 RectY = (u16)myInfo.Rect_Axis[1];
	
	bdc_msg[1] = CorpID;
	bdc_msg[3] = (u8)RectX;
	bdc_msg[4] = (u8)(RectX >> 8);
	bdc_msg[5] = (u8)RectY;
	bdc_msg[6] = (u8)(RectY >> 8);
	
	/* Start transmission */
	/*
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_setautorxreenable(0); //��ֹ�Զ���������
	dwt_setdblrxbuffmode(1); //����˫��ģʽ
			
	//Ҫ��HSRBP == ICRBP
	dwt_syncrxbufptrs();
	
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0); //�������Ҫ����һ�ν��յ�timeoutʱ��
	
	dwt_writetxdata(sizeof(bdc_msg), bdc_msg, 0);
	dwt_writetxfctrl(sizeof(bdc_msg), 0);
	
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	*/
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_setdblrxbuffmode(0); //close˫��ģʽ
			
	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0); //�������Ҫ����һ�ν��յ�timeoutʱ��
	
	dwt_writetxdata(sizeof(bdc_msg), bdc_msg, 0);
	dwt_writetxfctrl(sizeof(bdc_msg), 0);
	
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	Double_Buff_Recp_Listening(ID, 1);
}

void CorpInfo_Msg(u8 CorpID, u16 *Axis)
{
	//CmdType | CorpID | MyID | RectXL | RectXH | RectYL | RectYH | CRC | CRC
	static u8 bdc_msg[9] = {0x05, 0xFF, 0, 0, 0, 0, 0, 0x0D, 0x0A};
	
	bdc_msg[2] = CorpID;
	bdc_msg[3] = (u8)Axis[0];
	bdc_msg[4] = (u8)(Axis[0] >> 8);
	bdc_msg[5] = (u8)Axis[1];
	bdc_msg[6] = (u8)(Axis[1] >> 8);
	
	/* Start transmission */
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_setdblrxbuffmode(0); //��ֹ˫��ģʽ
	
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
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
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_setdblrxbuffmode(0); //��ֹ˫��ģʽ
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_writetxdata(sizeof(slc_msg), slc_msg, 0);
	dwt_writetxfctrl(sizeof(slc_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void Boss_Msg(void)
{
	//CmdType | 0x0D | 0x0A
	static u8 boss_msg[3] = {0x06, 0x0D, 0x0A};
	
	/* Start transmission */
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_setdblrxbuffmode(0); //��ֹ˫��ģʽ
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_writetxdata(sizeof(boss_msg), boss_msg, 0);
	dwt_writetxfctrl(sizeof(boss_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}

void Resp_Msg(void)
{
	//CmdType | MyID | ReserveForCounter | CRC | CRC
	static u8 resp_msg[5] = {0x07, MyID, 0, 0x0D, 0x0A};
	
	/* Start transmission */
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	dwt_setdblrxbuffmode(0); //��ֹ˫��ģʽ
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_writetxdata(sizeof(resp_msg), resp_msg, 0);
	dwt_writetxfctrl(sizeof(resp_msg), 0);
	dwt_starttx(DWT_START_TX_IMMEDIATE);
}
