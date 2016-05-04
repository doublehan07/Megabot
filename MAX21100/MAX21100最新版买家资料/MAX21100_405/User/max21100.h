#ifndef __MAX21100_H__
#define	__MAX21100_H__

#include "stm32f4xx.h"

#ifndef __MAX21100_C__
#define EXT_MAX21100 extern

#else 
#define EXT_MAX21100   	        


#endif


//BANK0
#define MAX21100_POWER_CFG      0x00 
#define MAX21100_GYRO_CFG1      0x01 
#define MAX21100_GYRO_CFG2      0x02 
#define MAX21100_GYRO_CFG3      0x03 
#define MAX21100_PWR_ACC_CFG      0x04 
#define MAX21100_ACC_CFG1      0x05 
#define MAX21100_ACC_CFG2        0x06 
#define MAX21100_MAG_SLV_CFG     0x07
#define MAX21100_MAG_SLV_ADD        0x08 
#define MAX21100_MAG_SLV_REG     0x09
#define MAX21100_MAG_MAP_REG        0x0A 
#define MAX21100_I2C_MST_ADD     0x0B
#define MAX21100_I2C_MST_DATA    0x0C 
#define MAX21100_MAG_OFS_X_MSB     0x0D
#define MAX21100_MAG_OFS_X_LSB     0x0E
#define MAX21100_MAG_OFS_Y_MSB     0x0F
#define MAX21100_MAG_OFS_Y_LSB     0x10 
#define MAX21100_MAG_OFS_Z_MSB     0x11
#define MAX21100_MAG_OFS_Z_LSB     0x12
#define MAX21100_DR_CFG           0x13
#define MAX21100_IO_CFG      0x14
#define MAX21100_I2C_PAD     0x15
#define MAX21100_I2C_CFG           0x16
#define MAX21100_FIFO_TH           0x17
#define MAX21100_FIFO_CFG          0x18
#define MAX21100_DSYNC_CFG     0x1A
#define MAX21100_DSYNC_CNT     0x1B
#define MAX21100_ITF_OTP          0x1C







//BANK1
#define MAX21100_INT_REF_X      0x00 
#define MAX21100_INT_REF_Y      0x01 
#define MAX21100_INT_REF_Z      0x02 
#define MAX21100_INT_DEB_X      0x03 
#define MAX21100_INT_DEB_Y      0x04 
#define MAX21100_INT_DEB_Z      0x05 
#define MAX21100_INT_MSK_X        0x06 
#define MAX21100_INT_MSK_Y     0x07
#define MAX21100_INT_MSK_Z        0x08 
#define MAX21100_INT_MASK_AO     0x09
#define MAX21100_INT_CFG1        0x0A 
#define MAX21100_INT_CFG2     0x0B
#define MAX21100_INT_TMO    0x0C 
#define MAX21100_INT_STS_UL    0x0D
#define MAX21100_INT_TST    0x0E
#define MAX21100_INT_MST     0x0F
#define MAX21100_INT_SRC_SEL           0x17
#define MAX21100_SERIAL_5     0x1A
#define MAX21100_SERIAL_4     0x1B
#define MAX21100_SERIAL_3          0x1C
#define MAX21100_SERIAL_2     0x1D
#define MAX21100_SERIAL_1     0x1E
#define MAX21100_SERIAL_0          0x1F






//BANK2
#define MAX21100_QUAT0_H      0x00 
#define MAX21100_QUAT0_L      0x01 
#define MAX21100_QUAT1_H      0x02 
#define MAX21100_QUAT1_L      0x03 
#define MAX21100_QUAT2_H      0x04 
#define MAX21100_QUAT2_L      0x05 
#define MAX21100_QUAT3_H      0x06 
#define MAX21100_QUAT3_L      0x07
#define MAX21100_BIAS_GYRO_X_H           0x13
#define MAX21100_BIAS_GYRO_X_L      0x14
#define MAX21100_BIAS_GYRO_Y_H     0x15
#define MAX21100_BIAS_GYRO_Y_L           0x16
#define MAX21100_BIAS_GYRO_Z_H           0x17
#define MAX21100_BIAS_GYRO_Z_L          0x18
#define MAX21100_BIAS_ACC_X     0x19
#define MAX21100_BIAS_ACC_Y     0x1A
#define MAX21100_BIAS_ACC_Z     0x1B
#define MAX21100_FUS_CFG0     0x1C
#define MAX21100_FUS_CFG1     0x1D
#define MAX21100_GYRO_ODR_TRIM    0x1F





#define MAX21100_WHO_AM_I          0x20
#define MAX21100_REVISION_ID        0x21
#define MAX21100_BANK_SELECT        0x22
#define MAX21100_SYSTEM_STATUS          0x23
#define MAX21100_GYRO_X_H     0x24
#define MAX21100_GYRO_X_L    0x25
#define MAX21100_GYRO_Y_H     0x26
#define MAX21100_GYRO_Y_L    0x27
#define MAX21100_GYRO_Z_H   0x28
#define MAX21100_GYRO_Z_L     0x29
#define MAX21100_ACC_X_H    0x2A
#define MAX21100_ACC_X_L    0x2B
#define MAX21100_ACC_Y_H      0x2C
#define MAX21100_ACC_Y_L     0x2D
#define MAX21100_ACC_Z_H     0x2E
#define MAX21100_ACC_Z_L     0x2F
#define MAX21100_MAG_X_H     0x30
#define MAX21100_MAG_X_L    0x31
#define MAX21100_MAG_Y_H     0x32
#define MAX21100_MAG_Y_L      0x33
#define MAX21100_MAG_Z_H    0x34
#define MAX21100_MAG_Z_L     0x35
#define MAX21100_TEMP_H   0x36
#define MAX21100_TEMP_L     0x37
#define MAX21100_FIFO_COUNT     0x3C
#define MAX21100_FIFO_STATUS    0x3D
#define MAX21100_FIFO_DATA    0x3E
#define MAX21100_RST_REG     0x3F



//寄存器值
#define MAX21100_POWER_CFG_DATA       0x7F                   //全部上电启动
#define MAX21100_ACC_CFG2_DATA        0X0A
#define MAX21100_MAG_SLV_CFG_DATA     0X86                   //开启外接MAG，6位数据长度
#define MAX21100_MAG_SLV_ADD_DATA     0x1E									 //7位外接MAG IIC地址
#define MAX21100_MAG_SLV_REG_DATA     0X03                   //外接MAG寄存器地址
#define MAX21100_MAG_MAP_REG_DATA     0X08

#define MAX21100_I2C_MST_ADD_DATA     0X02                   
#define MAX21100_I2C_MST_DATA_DATA    0X00									 
#define MAX21100_DR_CFG_DATA          0X01                   //MAX21100主IIC控制外接罗盘
#define MAX21100_I2C_CFG_DATA          0X01



#define CS_MAX0    GPIOC->BSRRH=GPIO_Pin_4
#define CS_MAX1    GPIOC->BSRRL=GPIO_Pin_4



typedef struct{

		int16_t X;
		int16_t Y;
		int16_t Z;
				
}MAX21100_TYPE;




EXT_MAX21100 MAX21100_TYPE		MAX21100_ACC_LAST,MAX21100_GYRO_LAST,MAX21100_MAG_LAST;		//最新一次读取值
EXT_MAX21100 MAX21100_TYPE		MAX21100_GYRO_OFFSET,MAX21100_ACC_OFFSET;			//零漂


EXT_MAX21100 void MAX21100_getMotion6(void);
 
EXT_MAX21100 uint8_t GET_MAX21100_ID(void); 


EXT_MAX21100 void MAX21100_Init(void);




#endif

