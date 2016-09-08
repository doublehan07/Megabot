#时钟配置
/* HSE=8MHz

 * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz*/

/* PLLCLK = HSE(8M)/8*288/4 = 72 MHz   */

/* Select PLL as system clock source   */

SPI3 = SPI_DW maximum = 20M，low_speed = less than 2M

Systick = 1ms

#版本说明
原计划在DW一块板子上完成主控功能。现在有电机控制接口、串口只差中断配置（九轴）、DW可以测距（未clean）