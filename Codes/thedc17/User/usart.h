#ifndef __USART_H
#define __USART_H
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "strategy.h"
#include <stm32f4xx.h>
#include <stdio.h>

void USART2_Configuration(void);
void USART2_puts(char*);
void Usart2Put(uint8_t ch);
uint8_t Usart2Get(void);


void USART5_puts(char*);
void USART5_Configuration(void);
void Usart5Put(uint8_t ch);
uint8_t Usart5Get(void);

#endif
