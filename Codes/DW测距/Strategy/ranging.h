/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RANGING_H
#define __RANGING_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "communication.h"
#include "deca_regs.h"
#include "deca_device_api.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern double distance;
extern uint16_t distance_cm;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Private functions ---------------------------------------------------------*/
uint8_t Initiator_Communication(uint8_t TargetID);
uint16_t Receptor_Communication(void);

#endif /* __RANGING */
