
#define __LSM6DS3_C__

#include "main.h"





void LSM6DS3_CS_Config(void)
{						         	 

	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO��ʼ���ṹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//����ʱ��ʹ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//��ӦGPIO����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//״̬Ϊ���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO�ٶ�Ϊ50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	CS_LSM1;

		
}





u8 LSM6DS3_writeReg(unsigned char reg, unsigned char val) 
{
	u8 status;	
	CS_LSM0;          //ʹ��SPI����	    
	status =SPI1_ReadWriteByte(reg);//���ͼĴ����� 
	SPI1_ReadWriteByte(val);      //д��Ĵ�����ֵ
	CS_LSM1; //��ֹSPI����	   
	return (status);       			//����״ֵ̬
}






u8 LSM6DS3_Read_Reg(u8 reg)
{
	u8 reg_val;	    
	CS_LSM0;          //ʹ��SPI����		
	SPI1_ReadWriteByte(reg+0X80);   //���ͼĴ�����
	reg_val=SPI1_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
	CS_LSM1; //��ֹSPI����	    
	return (reg_val);           //����״ֵ̬
}	






u8 GET_LSM6DS3_ID(void) 
{
  return LSM6DS3_Read_Reg(0X0F);  //LSM6DS3

}   


void LSM6DS3_Init(void)
{

		LSM6DS3_CS_Config();
	
	  LSM6DS3_writeReg(LSM6DS3_CTRL1_XL, 0X60) ;
		LSM6DS3_writeReg(LSM6DS3_CTRL2_G, 0X78) ;
		delay_ms(200);	

}


void LSM6DS3_getMotion6()
{
  

	
	LSM6DS3_GYRO_LAST.X=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_GYRO_X_H)) << 8) | LSM6DS3_Read_Reg(LSM6DS3_GYRO_X_L)) ;
	LSM6DS3_GYRO_LAST.Y=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_GYRO_Y_H)) << 8) | LSM6DS3_Read_Reg(LSM6DS3_GYRO_Y_L)) ;
	LSM6DS3_GYRO_LAST.Z=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_GYRO_Z_H)) << 8) | LSM6DS3_Read_Reg(LSM6DS3_GYRO_Z_L)) ;
	
	LSM6DS3_ACC_LAST.X=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_ACC_X_H)) << 8)   | LSM6DS3_Read_Reg(LSM6DS3_ACC_X_L)) ;
	LSM6DS3_ACC_LAST.Y=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_ACC_Y_H)) << 8)   | LSM6DS3_Read_Reg(LSM6DS3_ACC_Y_L)) ;
	LSM6DS3_ACC_LAST.Z=((((int16_t)LSM6DS3_Read_Reg(LSM6DS3_ACC_Z_H)) << 8)   | LSM6DS3_Read_Reg(LSM6DS3_ACC_Z_L)) ;

	
}











//------------------End of File----------------------------


