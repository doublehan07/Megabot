/**
  ******************************************************************************
  * DW1000之SPI
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "SPI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void SPIInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	
	//SPI1 PA6:MISO PA7:MOSI PA5:SCK PA4/PA15:NSS
	//PA6:MISO PA7:MOSI PA5:SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	
//	//SPI2 PC2/PB14:MISO PC3/PB15:MOSI PB10/PB13/PD3:SCK PB12/PB9:NSS
//	//PB14:MISO PB15:MOSI PB13:SCK
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  //GPIO_Init(GPIOB, &GPIO_InitStructure);
//	//GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
//	//GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
//	//GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);
//	
//	//SPI3 PC11:MISO PC12/PD6:MOSI PC10:SCK PA4/PA15:NSS
//	//PC11:MISO PC12:MOSI PC10:SCK
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  //GPIO_Init(GPIOC, &GPIO_InitStructure);
//	//GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3);
//	//GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);
//	//GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);
	
	//片选
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//工作模式 SPI_Direction_2Lines_FullDuplex，SPI_Direction_2Lines_RxOnly等 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主从模式选择 SPI_Mode_Master，SPI_Mode_Slave
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;//数据位选择 SPI_DataSize_8b，SPI_DataSize_16b
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//时钟空闲电平选择 SPI_CPOL_High，SPI_CPOL_Low
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//数据捕捉跳变沿选择 SPI_CPHA_2Edge，SPI_CPHA_1Edge 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//NSS信号由硬件还是软件控制 SPI_NSS_Soft，SPI_NSS_Hard 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//时钟分频选择
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据大小端选择 SPI_FirstBit_MSB,SPI_FirstBit_LSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);//写入配置信息
	SPI_Cmd(SPI1, ENABLE);
	SPI_SSOutputCmd(SPI1, DISABLE);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
}

//先写个阻塞的，SPI速度较快，理论上应该不会有问题，且与mbed内的write函数效果相同
#define SPIx SPI1
uint8_t SPI_RW(uint8_t TxData)
{
	u16 retry = 0;
	while(!(SPIx->SR&SPI_I2S_FLAG_TXE))//TXE
	{
		retry++;
		if (retry >= 0xFFFE) return 0;
	}
	SPIx->SR&=~SPI_I2S_FLAG_RXNE;
	retry = 0;
	SPIx->DR = TxData;
	while(!(SPIx->SR&SPI_I2S_FLAG_RXNE))//RXNE
	{
		retry++;
		if (retry >= 0xFFFE) return 0;
	}
	return SPIx->DR;
}

void select() {
    //irq.disable_irq();
		//假定使用SPI1，当然这只是普通的IO
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
}
void deselect() {
    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
    //irq.enable_irq();
}
