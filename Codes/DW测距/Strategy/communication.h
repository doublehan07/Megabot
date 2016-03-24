/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <string.h>

#include "port.h"
#include "ranging.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	u8 CmdType;
	u8 ID;
}RX_Command;

typedef struct
{
	u8 TargetID;
	u16 Dist;
}Parse_DW_Data;

/* Exported constants --------------------------------------------------------*/
extern unsigned char ucRxBuffer[5];
extern RX_Command upperCmd;
extern __IO uint8_t Usart_RX_flag;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Private functions ---------------------------------------------------------*/
void ParseSerialData(unsigned char ucData);
void Ranging_Stategy(void);

#endif /* __COMMUNICATION */
