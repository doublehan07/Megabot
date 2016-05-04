#define __SPI_C__




#include "main.h"


/******************************************************************
								SPI1 读写一个字节
								TxData:要写入的字节
								返回值:读取到的字节
******************************************************************/
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry>200)return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry>200)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据					    
}



/******************************************************************
									  SPI2设为从机初始化
******************************************************************/
void SPI1_Slave_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;     
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct); 
	
	SPI_Cmd(SPI1, ENABLE); // enable SPI1
	
}  


/******************************************************************
									  SPI1设为主机初始化
******************************************************************/
void SPI1_Master_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;        // clock is high when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;      // data sampled at second edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft ; //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct); 
	
	SPI_Cmd(SPI1, ENABLE); // enable SPI1

}  








