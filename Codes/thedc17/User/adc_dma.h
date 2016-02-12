#ifndef _ADC_DMA_H
#define _ADC_DMA_H
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define ADC2_DR_ADDRESS     ((uint32_t)0x4001214C)
#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)

void ADC3_CH12_DMA_Config(void);
void ADC1_CH4_CH5_DMA_Config(void);
#endif
