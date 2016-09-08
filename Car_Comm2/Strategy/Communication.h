#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f10x.h"

#define PI 3.1415926

#define LINE_THRESHOLD 0.96

#define COMM_WAIT_TIME 25
#define CMD_SEND_WAIT_TIME 5
#define COMM_TIMEOUT 60
#define RANGING_TIMEOUT 100
#define RANGING_TIME 1
#define USART_TIMEOUT_0 1000
#define USART_TIMEOUT	500

#define RATIO_TEST_STABLE	4
#define RATIO_TEST_MOVE	2

#define INIT_RANGE_COUNT 10

#define RANGE_CMD_1_0 0x01
#define RANGE_CMD_2_0 0x02
#define RANGE_CMD_3_0 0x03
#define RANGE_CMD_1_2 0x12
#define RANGE_CMD_2_3 0x23
#define RANGE_CMD_3_1 0x31
#define BRD_COOR_CMD 0x41

u8 Run_Loop(void);

#endif
