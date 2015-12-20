#include "15W204s.h"
#include "SPI.h"
#include "DW1000.h"

sbit REC=P1^5;

#define UB_Length 10

idata uchar UP_front = 0, UP_tail = 0, TxFree = 1;
idata uchar UARTBUF[UB_Length];

/***************************????****************/
void UartInit(void)		//115200bps@30.000MHz
{
	SCON = 0x50;
	AUXR |= 0x04;
	T2L = 0xBF;
	T2H = 0xFF;
	AUXR |= 0x01;
	AUXR |= 0x10;
}


void GPIOInit(void)
{
	REC = 0;
}

void UART(void) interrupt 4
{
	if (TI)
	{
		TI = 0;
		if (UP_front != UP_tail)
		{
			SBUF = UARTBUF[UP_front++];
			UP_front %= UB_Length;
		}
		else
			TxFree = 1;
	}
}

void Timer0Init(void)
{
	TMOD = 0x00;
	AUXR &= 0x7F;
	TH0 = 19200/256;
	TL0 = 19200%256;
	ET0 = 1;
	TR0 = 1;
}

void UART_Write(uchar *d, uchar length)
{
	uchar i = 0;
	if(TxFree)
	{
		TxFree = 0;
		SBUF = d[i++];
	}
	while (i < length)
	{
		UARTBUF[UP_tail++] = d[i++];
		UP_tail %= UB_Length;
	}
}

main()
{
	UartInit();
	SPIInit();
	GPIOInit();
	//Timer0Init();
	EA = 1;
	ES = 1;
	DW1000_Init();
	while(1)
	{
		uint32_t buffer32;
		uint16_t buffer16;
		REC = !REC;
		while (!TxFree);//确保上一次数据已经发送完毕，保证读取不会比UART更快
		buffer16 = readRegister16(DW1000_DRX_CONF, 0x06);
		UART_Write((uchar *)&buffer16,2);
		buffer32 = readRegister32(DW1000_DRX_CONF, 0x08);
		UART_Write((uchar *)&buffer32,4);
	}
}
