#时钟配置

HSE = 8MHz

HCLK(AHB） = 168MHz, PCLK2(APB2) = 84MHz, PCLK1(APB1) = 42MHz 	

PLLCLK = HSE(8M) / 8 * 336 / 2 = 168MHz，PLL_Q = 7

Select PLL as system clock source

Systick = 1ms