



#ifndef __USART1_H__
#define	__USART1_H__

#include "stm32f4xx.h"

#ifndef __USART1_C__
#define EXT_USART1 extern

#else 
#define EXT_USART1   	        


#endif



EXT_USART1 void USART1_Configuration(void);             //串口初始化函数



int fputc(int ch, FILE *f);    //fputc重定向
int fgetc(FILE *f); //fgetc重定向

#endif






