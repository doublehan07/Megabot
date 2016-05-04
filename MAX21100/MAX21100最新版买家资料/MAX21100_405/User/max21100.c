
#define __MAX21100_C__

#include "main.h"



	

/********************************************************************
                      MAX21100 CS管脚端口配置
********************************************************************/
static void MAX21100_CS_Config(void)
{						         	 

	GPIO_InitTypeDef  GPIO_InitStructure;//GPIO初始化结构体
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//外设时钟使能
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//对应GPIO引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//状态为输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO速度为50MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	CS_MAX1;
	
}


/**************************实现函数********************************************
*函数原型:	   u8 MAX21100_writeReg(unsigned char reg, unsigned char val)
*功　　能:	   写MAX21100的寄存器
输入参数：    reg  寄存器地址
			  val   要写入的值	
输出参数：  无意义
*******************************************************************************/
u8 MAX21100_writeReg(unsigned char reg, unsigned char val) 
{
	u8 status;	
	CS_MAX0;          //使能SPI传输	    
	status =SPI1_ReadWriteByte(reg);//发送寄存器号 
	SPI1_ReadWriteByte(val);      //写入寄存器的值
	CS_MAX1; //禁止SPI传输	   
	return (status);       			//返回状态值
}



u8 MAX21100_Read_Reg(u8 reg)
{
	u8 reg_val;	    
	CS_MAX0;          //使能SPI传输		
	SPI1_ReadWriteByte(reg+0X80);   //发送寄存器号
	reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
	CS_MAX1; //禁止SPI传输	    
	return (reg_val);           //返回状态值
}	



/**************************实现函数********************************************
*函数原型:	  void HMC59X3_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
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
    
		//关闭从IIC
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


