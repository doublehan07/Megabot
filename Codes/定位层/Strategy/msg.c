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
	dwt_forcetrxoff(); //�������DW�ص�IDLE״̬������timeout��ʧЧ��֮���粻�ܽ��ܻῨ����whileѭ����
	
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR); //clear good&bad event
	
	dwt_writetxdata(sizeof(bdc_msg), bdc_msg, 0);
	dwt_writetxfctrl(sizeof(bdc_msg), 0);
	
	if(if_bdc_axis)
	{
		dwt_starttx(DWT_START_TX_IMMEDIATE);
	}
	else //�㲥��Ϣ���ڴ���Ӧ�ģ���Ҫ��¼��������ǰ������
	{
		//��Ҫ��˫��ģʽ���Զ��������գ�
		//˫��ģʽoverrun��case̫�Ѵ����ˣ�
		
		dwt_setautorxreenable(0); //��ֹ�Զ���������
		dwt_setdblrxbuffmode(1); //����˫��ģʽ
		
		//Ҫ��HSRBP == ICRBP
		dwt_syncrxbufptrs();
		
		dwt_setrxtimeout(0); //����ʽ�ȴ�
		
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
			dwt_syncrxbufptrs(); //�л���֤�������
			dwt_rxenable(0); //���Ͽ�ʼ������һ��
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
