#ifndef __LSM6DS3_H__
#define	__LSM6DS3_H__

#include "stm32f4xx.h"

#ifndef __LSM6DS3_C__
#define EXT_LSM6DS3 extern

#else 
#define EXT_LSM6DS3   	        


#endif




#define	 LSM6DS3_WHO_AM_I	  0x0F	
#define	 LSM6DS3_CTRL1_XL   0x10
#define	 LSM6DS3_CTRL2_G	  0x11
#define	 LSM6DS3_CTRL3_C  0x12
#define	 LSM6DS3_CTRL4_C	  0x13
#define	 LSM6DS3_CTRL5_C   0x14
#define	 LSM6DS3_CTRL6_C  0x15
#define	 LSM6DS3_CTRL7_C  0x16



#define	 LSM6DS3_CTRL8_XL  0x17
#define	 LSM6DS3_CTRL9_XL  0x18
#define	 LSM6DS3_CTRL10_C  0x19


#define	LSM6DS3_TEMP_H		0x21
#define	LSM6DS3_TEMP_L		0x20

#define	LSM6DS3_GYRO_X_H		0x23
#define	LSM6DS3_GYRO_X_L		0x22	
#define	LSM6DS3_GYRO_Y_H		0x25
#define	LSM6DS3_GYRO_Y_L		0x24
#define	LSM6DS3_GYRO_Z_H		0x27
#define	LSM6DS3_GYRO_Z_L		0x26



#define	LSM6DS3_ACC_X_H	0x29
#define	LSM6DS3_ACC_X_L	0x28
#define	LSM6DS3_ACC_Y_H	0x2B
#define	LSM6DS3_ACC_Y_L	0x2A
#define	LSM6DS3_ACC_Z_H	0x2D
#define	LSM6DS3_ACC_Z_L	0x2C



#define CS_LSM0    GPIOC->BSRRH=GPIO_Pin_4
#define CS_LSM1    GPIOC->BSRRL=GPIO_Pin_4



typedef struct{

		int16_t X;
		int16_t Y;
		int16_t Z;
				
}LSM6DS3_TYPE;



EXT_LSM6DS3 LSM6DS3_TYPE		LSM6DS3_ACC_LAST,LSM6DS3_GYRO_LAST;		//最新一次读取值
EXT_LSM6DS3 LSM6DS3_TYPE		LSM6DS3_GYRO_OFFSET,LSM6DS3_ACC_OFFSET;			//零漂



EXT_LSM6DS3 void LSM6DS3_CS_Config(void);

EXT_LSM6DS3 void LSM6DS3_getMotion6(void);
 
EXT_LSM6DS3 uint8_t GET_LSM6DS3_ID(void); 

EXT_LSM6DS3 void LSM6DS3_Init(void);




#endif

