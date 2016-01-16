#include "SPI.h"

sbit CE=P1^4;
sbit CSN=P1^0;
sbit MOSI=P1^1;
sbit MISO=P1^2;
sbit SCLK=P1^3;

void Delay(unsigned int t)
{
  unsigned int y;
  for(;t>0;t--)
   for(y=16;y>0;y--);
}

void SPIInit(void)
{
	SCLK = 0;
	MISO = 1;

	CE = 0;
	CSN = 1;
	Delay(5);
}

uchar SPI_RW(uchar date)
{
    uchar i;
   	for(i=0;i<8;i++)
   	{
	    if(date&0x80)
				MOSI=1;
	    else
				MOSI=0;
   	  date<<=1;
   	  SCLK=1;
		  if(MISO)
   	    date|=0x01;
   	  SCLK=0;
   	}
    return(date);
}

void select() {
    //irq.disable_irq();
    CSN = 0;
}
void deselect() {
    CSN = 1;
    //irq.enable_irq();
}