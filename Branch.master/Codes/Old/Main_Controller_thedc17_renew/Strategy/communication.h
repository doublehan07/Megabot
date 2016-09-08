/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <string.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	u8 TargetID;
	u16 Dist;
}Parse_DW_Data;

/* Exported constants --------------------------------------------------------*/
extern unsigned char DW_RX_Buffer[6];

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void Parse_DW1000_Data(unsigned char ucData);

#endif /* __COMMUNICATION */
