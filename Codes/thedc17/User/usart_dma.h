#ifndef __USART_DMA_H
#define __USART_DMA_H
#include "stm32f4xx_usart.h"
#include <stdbool.h>
void init_uart(void);
void drv_tx(uint8_t* , uint16_t);
void drv_gsm_deal_irq_dma_tx(void);
uint8_t drv_gsm_deal_irq_tx_end(void);
uint8_t drv_gsm_deal_irq_rx_end(uint8_t *);
void DMA1_Stream6_IRQHandler(void);
void USART2_IRQHandler(void);
void gsm_dma_tx_irq_handler(void);
void gsm_irq_handler(void);
void inf_gsm_deal_irq_dma_tx(void);
uint8_t inf_gsm_deal_irq_tx_end(void);
uint8_t inf_gsm_deal_irq_rx_end(uint8_t *);
#endif
