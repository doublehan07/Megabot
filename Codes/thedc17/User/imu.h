#ifndef __IMU_H
#define __IMU_H

#include "stdint.h"

static char rev_buf[64]; /* 接收到的原始数据 */

enum input_status { 
STATUS_IDLE, /* 空闲 */
STATUS_SOF, /* 接收到帧头 (0x88, 0xAF)*/
STATUS_LEN, /* 接收到长度字节 */
STATUS_DATA, /*接收到数据 */
};

struct imu_data { 
int16_t accl[3];
int16_t gyro[3];
int16_t mag[3];
int16_t yaw;
int16_t pitch;
int16_t roll;
int32_t presure;
};



int imu_rev_get_data(struct imu_data* data);

void imu_rev_process(char ch);

#endif
