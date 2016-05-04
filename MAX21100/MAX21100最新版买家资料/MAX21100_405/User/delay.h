


#ifndef __DELAY_H__
#define	__DELAY_H__

#include "stm32f4xx.h"

#ifndef __DELAY_C__
#define EXT_DELAY extern

#else 
#define EXT_DELAY  

#endif	        




//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й���
//����delay_us,delay_ms

EXT_DELAY void delay_init(u8 SYSCLK);
EXT_DELAY void delay_ms(u16 nms);
EXT_DELAY void delay_us(u32 nus);

#endif































