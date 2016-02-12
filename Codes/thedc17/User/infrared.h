#ifndef __INFRARED_H
#define __INFRARED_H
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx.h"
#include "stm32f4xx_syscfg.h"
#include <stdio.h>
void Infrared_GPIO_Config(void);
void NVIC_Config(void);
#endif
