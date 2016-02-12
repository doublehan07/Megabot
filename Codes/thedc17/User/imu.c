#include "imu.h"

struct imu_data IMU_Data;


void imu_rev_process(char ch) {
	static int len = 0;
	static int i; 
	static enum input_status status = STATUS_IDLE; /* running status machine */
	switch(status) 
		{ 
			case STATUS_IDLE: 
				if((uint8_t)ch == 0x88) { 
				status =STATUS_SOF; 
				} 
				break;
			case STATUS_SOF: if((uint8_t)ch == 0xAF) { status = STATUS_LEN; } break;
			case STATUS_LEN: len = ch; status = STATUS_DATA; i = 0; break;
			case STATUS_DATA:
				if(i == len)
				{ 
					status = STATUS_IDLE; 
					imu_rev_get_data(&IMU_Data); break; 
				}
				rev_buf[i++] = ch; break;
			default: break; 
		}
}

int imu_rev_get_data(struct imu_data* data){
	data->accl[0] = (rev_buf[0]<<8) + rev_buf[1];
	data->accl[1] = (rev_buf[2]<<8) + rev_buf[3];
	data->accl[2] = (rev_buf[4]<<8) + rev_buf[5];
	data->gyro[0] = (rev_buf[6]<<8) + rev_buf[7];
	data->gyro[1] = (rev_buf[8]<<8) + rev_buf[9];
	data->gyro[2] = (rev_buf[10]<<8) + rev_buf[11];
	data->mag[0] = (rev_buf[12]<<8) + rev_buf[13];
	data->mag[1] = (rev_buf[14]<<8) + rev_buf[15];
	data->mag[2] = (rev_buf[16]<<8) + rev_buf[17];
	data->roll = (rev_buf[18]<<8) + rev_buf[19];
	data->pitch = (rev_buf[20]<<8) + rev_buf[21];
	data->yaw = (rev_buf[22]<<8) + rev_buf[23];
	data->presure = (rev_buf[27]<<24) + (rev_buf[26]<<16) + (rev_buf[25]<<8) +
	(rev_buf[24]<<0);
	//printf("P/R/Y/P:%05d %05d %05d %05d\r",data->pitch/100, data->roll/100, data->yaw/10, data->presure);
	return 0;
}
