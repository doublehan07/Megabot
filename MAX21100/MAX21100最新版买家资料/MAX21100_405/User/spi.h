

#ifndef __SPI_H__
#define	__SPI_H__



#include "stm32f4xx.h"


#ifndef __SPI_C__
#define EXT_SPI extern

#else 
#define EXT_SPI   	        


#endif



EXT_SPI u8 SPI1_ReadWriteByte(u8 TxData);
EXT_SPI void SPI1_Slave_Init(void); 
EXT_SPI void SPI1_Master_Init(void);


		 


#endif









