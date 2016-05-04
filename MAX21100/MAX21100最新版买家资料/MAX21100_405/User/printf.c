


#define __PRINTF_C__


#include "main.h"




void  TO_PC(void)
{

	//MAX21100Êý¾Ý´òÓ¡				
	printf("ax:");
	printf("%d",MAX21100_ACC_LAST.X);
	printf("\r\n");
	
	printf("ay:");
	printf("%d",MAX21100_ACC_LAST.Y);
	printf("\r\n");
	
	printf("az:");
	printf("%d",MAX21100_ACC_LAST.Z);
	printf("\r\n");
	
	printf("gx:");
	printf("%d",MAX21100_GYRO_LAST.X);
	printf("\r\n");
	
	printf("gy:");
	printf("%d",MAX21100_GYRO_LAST.Y);
	printf("\r\n");
	
	printf("gz:");
	printf("%d",MAX21100_GYRO_LAST.Z);
	printf("\r\n");


	



}


