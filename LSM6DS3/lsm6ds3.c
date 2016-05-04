
#define __LSM6DS3_C__

#include "main.h"





void LSM6DS3_CS_Config(void)
{						         	 

	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO初始化结构体
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//外设时钟使能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//对应GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//状态为输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO速度为50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	CS_LSM1;

		
}





u8 LSM6DS3_writeReg(unsigned char reg, unsigned char val) 
{
	u8 status;	
	CS_LSM0;          //使能SPI传输	    
	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
	SPI1_ReadWriteByte(val);      //写入寄存器的值
	CS_LSM1; //禁止SPI传输	   
	return (status);       			//返回状态值
}






u8 LSM6DS3_Read_Reg(u8 reg)
{
	u8 reg_val;	    
	CS_LSM0;          //使能SPI传输		
	SPI1_ReadWriteByte(reg+0X80);   //发送寄存器号
	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
	CS_LSM1; //禁止SPI传输	    
	return (reg_val);           //返回状态值
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


