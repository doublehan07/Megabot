
#include "main.h"



void SYS_Init(void)
{
	
	
	delay_init(168);		     
		
  delay_ms(500);	
	
	USART1_Configuration();   
	
	SPI1_Master_Init();
	
	delay_ms(200);	        
	delay_ms(1000);	        


	MAX21100_Init();
	delay_ms(200);
	GET_MAX21100_ID();
	printf("MAX21100...IS.....OK\r\n");
	

	
}



int main(void)
{

	SYS_Init();
	
	while(1)
	{
		MAX21100_getMotion6();
		
		
		TO_PC();
		delay_ms(100);
	}
}


