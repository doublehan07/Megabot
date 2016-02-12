/*
 * =====================================================================================
 *
 *       Filename:  adc.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2013/10/25 16:23:56
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  madcc09 (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"

#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define ADC2_DR_ADDRESS     ((uint32_t)0x4001214C)
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

void ADC_Config(void);

