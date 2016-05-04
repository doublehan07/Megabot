
#define __MAX21100_C__

#include "main.h"



	

/********************************************************************
                      MAX21100 CS�ܽŶ˿�����
********************************************************************/
static void MAX21100_CS_Config(void)
{						         	 

	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO��ʼ���ṹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//����ʱ��ʹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//��ӦGPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//״̬Ϊ���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO�ٶ�Ϊ50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	CS_MAX1;
	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	   u8 MAX21100_writeReg(unsigned char reg, unsigned char val)
*��������:	   дMAX21100�ļĴ���
���������    reg  �Ĵ�����ַ
			  val   Ҫд���ֵ	
���������  ������
*******************************************************************************/
u8 MAX21100_writeReg(unsigned char reg, unsigned char val) 
{
	u8 status;	
	CS_MAX0;          //ʹ��SPI����	    
	status =SPI1_ReadWriteByte(reg);//���ͼĴ����� 
	SPI1_ReadWriteByte(val);      //д��Ĵ�����ֵ
	CS_MAX1; //��ֹSPI����	   
	return (status);       			//����״ֵ̬
}



u8 MAX21100_Read_Reg(u8 reg)
{
	u8 reg_val;	    
	CS_MAX0;          //ʹ��SPI����		
	SPI1_ReadWriteByte(reg+0X80);   //���ͼĴ�����
	reg_val=SPI1_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
	CS_MAX1; //��ֹSPI����	    
	return (reg_val);           //����״ֵ̬
}	



/**************************ʵ�ֺ���********************************************
*����ԭ��:	  void HMC59X3_getID(char id[3])
*��������:	   ��ȡоƬ��ID
���������     	ID��ŵ�����
���������  ��
*******************************************************************************/
u8 GET_MAX21100_ID(void) 
{
	return MAX21100_Read_Reg(MAX21100_WHO_AM_I);
}   


void MAX21100_Init(void)
{

		MAX21100_CS_Config();
		MAX21100_writeReg(MAX21100_POWER_CFG, MAX21100_POWER_CFG_DATA);
		delay_ms(100);
    
		//�رմ�IIC
		MAX21100_writeReg(MAX21100_I2C_CFG, MAX21100_I2C_CFG_DATA);
		delay_ms(100);	


}


void MAX21100_getMotion6()
{
  
	MAX21100_GYRO_LAST.X=((((int16_t)MAX21100_Read_Reg(MAX21100_GYRO_X_H)) << 8) | MAX21100_Read_Reg(MAX21100_GYRO_X_L)) ;
	MAX21100_GYRO_LAST.Y=((((int16_t)MAX21100_Read_Reg(MAX21100_GYRO_Y_H)) << 8) | MAX21100_Read_Reg(MAX21100_GYRO_Y_L)) ;
	MAX21100_GYRO_LAST.Z=((((int16_t)MAX21100_Read_Reg(MAX21100_GYRO_Z_H)) << 8) | MAX21100_Read_Reg(MAX21100_GYRO_Z_L)) ;
	
	MAX21100_ACC_LAST.X=((((int16_t)MAX21100_Read_Reg(MAX21100_ACC_X_H)) << 8)   | MAX21100_Read_Reg(MAX21100_ACC_X_L)) ;
	MAX21100_ACC_LAST.Y=((((int16_t)MAX21100_Read_Reg(MAX21100_ACC_Y_H)) << 8)   | MAX21100_Read_Reg(MAX21100_ACC_Y_L)) ;
	MAX21100_ACC_LAST.Z=((((int16_t)MAX21100_Read_Reg(MAX21100_ACC_Z_H)) << 8)   | MAX21100_Read_Reg(MAX21100_ACC_Z_L)) ;


	
}











//------------------End of File----------------------------


