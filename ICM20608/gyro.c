#include "gyro.h"

#ifdef GYRO_ICM20608
#define ICM20608_ADDR 0x68
#define ICM20608_ACCEL_DATA_REG 0x3B //6 bytes accel + 2 bytes temp + 6 bytes gyro
#define ICM20608_GYRO_DATA_REG 0x43
#endif

#ifdef GYRO_ICM20608
void icm20608_set_wakeup(void);
void icm20608_init(void)
{
	//unsigned char set_register;
	//unsigned char ret=0;
	unsigned char who_am_i;
	
	icm20608_set_wakeup();//assume gyro woking ing internal 8MHz;

	i2c_write(ICM20608_ADDR,0x6B,0x03);//wakeup

	//set Auxiliary I2C Supply Selection
	//ret += i2c_read_gyro(0x75,&who_am_i,1);
	
	i2c_read_bytes(ICM20608_ADDR, 0x75, &who_am_i,1);
	
	who_am_i = who_am_i&0xFF;

	while(0xAF != who_am_i)
	{
		//icm20608 not availabley
		delay_8ms_hard(60);
		LED= ~LED;
		
		i2c_read_bytes(ICM20608_ADDR, 0x75, &who_am_i,1);
		who_am_i = who_am_i&0xFF;	
	}

	//set SMPLRT_DIV
	//set the gyro register output rate
	//register_output_rate = analog_sample_rate/(divider+1);//analog_sample_rate = 1 KHz or 8 KHz depend on DLPF_CFG
	//set_register = 0x01;
	//ret += i2c_write_gyro(0x19,&set_register,1);//500Hz
	i2c_write(ICM20608_ADDR, 0x19, 0x07);//125Hz
	
	//set the DLPF_CFG and External Sync //diffrent form mpu3050
	//		|Register (Hex) |Register (Decimal) |Bit7 |Bit6 |Bit5 |Bit4 |Bit3 |Bit2 |Bit1 |Bit0 | Default Value 
	//		|1A		 		|26 				|           |  EXT_SYNC_SET   |      DLPF_CFG   | 00h
	//42Hz/4.8ms/1KHz
	//set_register = 0x03;//44Hz
	//ret += i2c_write_gyro(0x1A,&set_register,1); 
	i2c_write(ICM20608_ADDR, 0x1A,0x02);//94Hz

	//set the GYRO_CONFIG
	//set_register = 0x10;
	//Full range +-1000deg/s
	//ret += i2c_write_gyro(0x1B,&set_register,1); 
	//i2c_write(ICM20608_ADDR,0x1B,0x18);//+-2000deg/s
	i2c_write(ICM20608_ADDR,0x1B,0x18);//+-2000deg/s
	//set the ACCEL_CONFIG
	//Full range +-2g /DHPF rest
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x1C,&set_register,1); 
	i2c_write(ICM20608_ADDR,0x1C,0x08);//+-4G ;cutoff 5Hz
	//set the MOT_THR 
	//motion threshold xx?
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x1F,&set_register,1); 
	
	//set the MOT_DUR
	//motion duration xx?
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x20,&set_register,1); 
	
	//set the ZRMOT_THR
	//zero motion threshold xx?
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x21,&set_register,1); 
	
	//set the ZRMOT_DUR
	//zero motion duration xx?
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x22,&set_register,1); 
	
	
	//set the INT_CONFIG/Bypass CONFIG
	//high0/open drain1/50us long pulse/clear by readint INT_STATUS/bypass enbale/clock out disable
	//set_register = 0x42;
	//ret += i2c_write_gyro(0x37,&set_register,1); 

	//set the INT_ENABLE 
	//free fall disable/motion enable/zero motion enable/data ready disable
	//set_register = 0x60;
	//ret += i2c_write_gyro(0x38,&set_register,1); 

	//set the USER_CTRL 
	//
	//set_register = 0x00;
	//ret += i2c_write_gyro(0x6A,&set_register,1); 
	i2c_write(ICM20608_ADDR,0x6A,0x00);
	
	//set power management
	//select clk z-aix
	//set_register = 0x03;
	//ret += i2c_write_gyro(0x6B,&set_register,1); 
	i2c_write(ICM20608_ADDR,0x6B,0x03);

	//return ret;
	
}
void icm20608_read_data(unsigned char *pbuf_accel,unsigned char *pbuf_gyro)
{
	i2c_read_bytes(ICM20608_ADDR,ICM20608_ACCEL_DATA_REG,pbuf_accel,6);
	i2c_read_bytes(ICM20608_ADDR,ICM20608_GYRO_DATA_REG,pbuf_gyro,6);
}
void icm20608_set_sleep(void)
{	
	//unsigned char reg;
	i2c_write(ICM20608_ADDR,0x6B,0x00);//switch clock to internal 8MHz
	lib_delay_us(300);
	i2c_write(ICM20608_ADDR,0x6C,0x3F);//disable accels and gyros
	//i2c_write(ICM20608_ADDR,0x6B,0x43);//sleep
	i2c_write(ICM20608_ADDR,0x6B,0x40);// sleep on internal 8MHz
}
void icm20608_set_wakeup(void)
{	
	i2c_write(ICM20608_ADDR,0x6B,0x00);//wake up on internal 8MHz
	
	i2c_write(ICM20608_ADDR,0x6C,0x00);//enable gyros
	//lib_delay_us(300);
	lib_delay_us(800);
	i2c_write(ICM20608_ADDR,0x6B,0x01);//switch clock to gyro z aixs
	//lib_delay_us(300);
	lib_delay_us(800);
	
	//i2c_write(ICM20608_ADDR,0x6C,0x00);
	//icm20608_init();
}
#endif

void gyro_sleep(void)
{
	icm20608_set_sleep();
}
void gyro_wakeup(void)
{
	icm20608_set_wakeup();
}


void gyro_init(void)
{
	icm20608_init();	
}



