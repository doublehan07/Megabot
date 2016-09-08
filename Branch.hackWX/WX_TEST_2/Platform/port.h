/**
  ******************************************************************************
  * 接口定义
  ******************************************************************************
  */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

//For Systick
#ifdef CLOCKS_PER_SEC
	#undef CLOCKS_PER_SEC
#endif
#define CLOCKS_PER_SEC					1000

/* Exported functions ------------------------------------------------------- */
//Periph Init
void Our_Sys_Init(void);

void TimingDelay_Decrement(void);
void SetTick(u32 tick);
u32 GetTick(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
