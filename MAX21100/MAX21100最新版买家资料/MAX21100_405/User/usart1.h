



#ifndef __USART1_H__
#define	__USART1_H__

#include "stm32f4xx.h"

#ifndef __USART1_C__
#define EXT_USART1 extern

#else 
#define EXT_USART1   	        


#endif



EXT_USART1 void USART1_Configuration(void);             //���ڳ�ʼ������



int fputc(int ch, FILE *f);    //fputc�ض���
int fgetc(FILE *f); //fgetc�ض���

#endif






