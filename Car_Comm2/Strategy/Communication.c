#include "Communication.h"
#include "initiator.h"
#include "receptor.h"
#include "port.h"
#include "ranging_api.h"

#include <math.h>

//static u16 d01 = 611;
//static u16 d02 = 516;
//static u16 d03 = 780;
//static u16 d12 = 810;
//static u16 d13 = 600;
//static u16 d23 = 500;

static u16 d01;
static u16 d02;
static u16 d03;
static u16 d12;
static u16 d13;
static u16 d23;

static short angle_0 = 0;

static short dx[4] = {0, 0, 0, 0};
static short dy[4] = {0, 0, 0, 0};

static short x[4] = {0, 0, 0, 0};
static short y[4] = {0, 0, 0, 0};

static short target_x[4] = {0, 0, 0, 0};
static short target_y[4] = {0, 0, 0, 0};

static short shape_x[4] = {0, 0, 0, 0};
static short shape_y[4] = {0, 0, 0, 0};

static u8 success;
static u8 rcv_displacement;
static u8 init_range_count = 0;
static u8 shape_setted = 0;

static unsigned char ucRxCnt_dis = 0;
static unsigned char ucRxCnt_shape = 0;

static u8 Cmd_Msg_TX[4] = {0x7A, MyID, 0x00, 0x00};
static u8 Cmd_Msg_RX[4] = {0x7A, 0x00, MyID, 0x00};
static u8 Range_Info_TX[9] = {0x8A, MyID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 Range_Info_RX[9] = {0x8A, 0x00, MyID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 Coordinate_Info_TX[13] = {0x9A, MyID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 Coordinate_Info_RX[13] = {0x9A, 0x00, MyID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 Control_Cmd_TX[13] = {0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E};

float getAngleDiffRad(float angle1, float angle2)
{
	float angle_diff = angle1 - angle2;
	while(angle_diff > PI)
		angle_diff -= 2*PI;
	while(angle_diff < -PI)
		angle_diff += 2*PI;
	return angle_diff;
}

float getMin3(float x1, float x2, float x3)
{
	float tmp = (x1 < x2)? x1:x2;
	return (tmp < x3)? tmp:x3;
}

float getMax3(float x1, float x2, float x3)
{
	float tmp = (x1 > x2)? x1:x2;
	return (tmp > x3)? tmp:x3;
}

void Calculate()
{
	if(init_range_count < INIT_RANGE_COUNT)
	{
		static int sum_d01 = 0;
		static int sum_d02 = 0;
		static int sum_d03 = 0;
		static int sum_d12 = 0;
		static int sum_d13 = 0;
		static int sum_d23 = 0;
		init_range_count++;
		sum_d01 += d01;
		sum_d02 += d02;
		sum_d03 += d03;
		sum_d12 += d12;
		sum_d13 += d13;
		sum_d23 += d23;
		if(init_range_count == INIT_RANGE_COUNT)
		{
			float d1 = 0;
			float d2 = 0;
			float d3 = 0;
			d1 = 0.25 / INIT_RANGE_COUNT * (sum_d01+sum_d02+sum_d03+sum_d01-sum_d12-sum_d13);
			d2 = 0.25 / INIT_RANGE_COUNT * (sum_d01+sum_d02+sum_d03+sum_d02+sum_d12-sum_d23);
			d3 = 0.25 / INIT_RANGE_COUNT * (sum_d01+sum_d02+sum_d03+sum_d03+sum_d13+sum_d23);
			x[1] = -d1 * cos(angle_0*PI/180);
			y[1] = -d1 * sin(angle_0*PI/180);
			x[2] = -d2 * cos(angle_0*PI/180);
			y[2] = -d2 * sin(angle_0*PI/180);
			x[3] = -d3 * cos(angle_0*PI/180);
			y[3] = -d3 * sin(angle_0*PI/180);
			target_x[1] = x[1];
			target_y[1] = y[1];
			target_x[2] = x[2];
			target_y[2] = y[2];
			target_x[3] = x[3];
			target_y[3] = y[3];
		}
	}
	else
	{
		static float rot_mat[2][2] = {{0, 0}, {0, 0}};
		float temp_x[3] = {0, 0, 0};
		float temp_y[3] = {0, 0, 0};
		float temp_rot_x[3] = {0, 0, 0};
		float temp_rot_y[3] = {0, 0, 0};
		float fd[4][4] = {0};
		float theta[3] = {0, 0, 0};
		float tmp, tmp1, tmp2, i;
		float start, step, best, err, best_err;
		
		fd[0][1] = fd[1][0] = d01;
		fd[0][2] = fd[2][0] = d02;
		fd[0][3] = fd[3][0] = d03;
		fd[1][2] = fd[2][1] = d12;
		fd[1][3] = fd[3][1] = d13;
		fd[2][3] = fd[3][2] = d23;
		
		x[1] += dx[1] - dx[0];
		x[2] += dx[2] - dx[0];
		x[3] += dx[3] - dx[0];
		y[1] += dy[1] - dy[0];
		y[2] += dy[2] - dy[0];
		y[3] += dy[3] - dy[0];
		
		theta[0] = atan2(y[1], x[1]);
		theta[1] = atan2(y[2], x[2]);
		theta[2] = atan2(y[3], x[3]);
		
		// triangle 012
		tmp = (pow(fd[0][1], 2) + pow(fd[0][2], 2) - pow(fd[1][2], 2)) / (2*fd[0][1]*fd[0][2]);
		if(tmp < -LINE_THRESHOLD)
		{
			tmp = (fd[1][2] - fd[0][1] - fd[0][2]) / 3;
			temp_x[0] = fd[0][1] + tmp;
			temp_y[0] = 0;
			temp_x[1] = -(fd[0][2] + tmp);
			temp_y[1] = 0;
		}
		else if(tmp > LINE_THRESHOLD)
		{
			if(fd[0][1] > fd[0][2])
			{
				tmp = (fd[0][1] - fd[0][2] - fd[1][2]) / 3;
				temp_x[0] = fd[0][1] - tmp;
				temp_y[0] = 0;
				temp_x[1] = fd[0][2] + tmp;
				temp_y[1] = 0;
			}
			else
			{
				tmp = (fd[0][2] - fd[0][1] - fd[1][2]) / 3;
				temp_x[0] = fd[0][1] + tmp;
				temp_y[0] = 0;
				temp_x[1] = fd[0][2] - tmp;
				temp_y[1] = 0;
			}
		}
		else
		{
			temp_x[0] = fd[0][1];
			temp_y[0] = 0;
			temp_x[1] = fd[0][2] * tmp;
			temp_y[1] = sqrt(pow(fd[0][2], 2) - pow(temp_x[1], 2));
		}
		
		// triangle 013
		tmp = (pow(fd[0][1], 2) + pow(fd[0][3], 2) - pow(fd[1][3], 2)) / (2*fd[0][1]*fd[0][3]);
		if(tmp < -LINE_THRESHOLD)
		{
			tmp = (fd[1][3] - fd[0][1] - fd[0][3]) / 3;
			temp_x[2] = -(fd[0][3] + tmp);
			temp_y[2] = 0;
		}
		else if(tmp > LINE_THRESHOLD)
		{
			if(fd[0][1] > fd[0][3])
			{
				tmp = (fd[0][1] - fd[0][3] - fd[1][3]) / 3;
				temp_x[2] = fd[0][3] + tmp;
				temp_y[2] = 0;
			}
			else
			{
				tmp = (fd[0][3] - fd[0][1] - fd[1][3]) / 3;
				temp_x[2] = fd[0][3] - tmp;
				temp_y[2] = 0;
			}
		}
		else
		{
			temp_x[2] = fd[0][3] * tmp;
			temp_y[2] = sqrt(pow(fd[0][3], 2) - pow(temp_x[2], 2));
		}
		
		tmp1 = fabs(sqrt(pow(temp_x[2] - temp_x[1], 2) + pow(temp_y[2] - temp_y[1], 2)) - fd[2][3]);
		tmp2 = fabs(sqrt(pow(temp_x[2] - temp_x[1], 2) + pow(temp_y[2] + temp_y[1], 2)) - fd[2][3]);
		if(tmp1 > tmp2)
			temp_y[2] = -temp_y[2];
		
		if(temp_y[1] != 0)
		{
			if(getAngleDiffRad(theta[1], theta[0]) < 0)
			{
				temp_y[1] = -temp_y[1];
				temp_y[2] = -temp_y[2];
			}
		}
		if(temp_y[2] != 0)
		{
			if(temp_y[2] * getAngleDiffRad(theta[2], theta[0]) < 0)
			{
				temp_y[1] = -temp_y[1];
				temp_y[2] = -temp_y[2];
			}
		}
		
		i = 0;
		err = -1;
		start = getAngleDiffRad(theta[0], atan2(temp_y[0], temp_x[0]));
		tmp1 = getAngleDiffRad(theta[1], atan2(temp_y[1], temp_x[1]));
		tmp2 = getAngleDiffRad(theta[2], atan2(temp_y[2], temp_x[2]));
		tmp1 = getAngleDiffRad(tmp1, start);
		tmp2 = getAngleDiffRad(tmp2, start);
		start += getMin3(0, tmp1, tmp2);
		step = (getMax3(0, tmp1, tmp2) - getMin3(0, tmp1, tmp2)) / 20;
		best_err = -1;
		best = 0;
		for(i = 0; i < 20; i++)
		{
			tmp = start + step * i;
			rot_mat[0][0] = cos(tmp);
			rot_mat[0][1] = -sin(tmp);
			rot_mat[1][0] = sin(tmp);
			rot_mat[1][1] = cos(tmp);
			temp_rot_x[0] = rot_mat[0][0] * temp_x[0] + rot_mat[0][1] * temp_y[0];
			temp_rot_y[0] = rot_mat[1][0] * temp_x[0] + rot_mat[1][1] * temp_y[0];
			temp_rot_x[1] = rot_mat[0][0] * temp_x[1] + rot_mat[0][1] * temp_y[1];
			temp_rot_y[1] = rot_mat[1][0] * temp_x[1] + rot_mat[1][1] * temp_y[1];
			temp_rot_x[2] = rot_mat[0][0] * temp_x[2] + rot_mat[0][1] * temp_y[2];
			temp_rot_y[2] = rot_mat[1][0] * temp_x[2] + rot_mat[1][1] * temp_y[2];
			err = pow(temp_rot_x[0] - x[1], 2) + pow(temp_rot_y[0] - y[1], 2);
			tmp1 = pow(temp_rot_x[1] - x[2], 2) + pow(temp_rot_y[1] - y[2], 2);
			tmp2 = pow(temp_rot_x[2] - x[3], 2) + pow(temp_rot_y[2] - y[3], 2);
			err = err + tmp1 + tmp2;// - getMax3(err, tmp1, tmp2);
			if(best_err == -1 || err < best_err)
			{
				best_err = err;
				best = i;
			}
		}
		tmp = start + step * best;
		rot_mat[0][0] = cos(tmp);
		rot_mat[0][1] = -sin(tmp);
		rot_mat[1][0] = sin(tmp);
		rot_mat[1][1] = cos(tmp);
		tmp = rot_mat[0][0] * temp_x[0] + rot_mat[0][1] * temp_y[0];
		temp_y[0] = rot_mat[1][0] * temp_x[0] + rot_mat[1][1] * temp_y[0];
		temp_x[0] = tmp;
		tmp = rot_mat[0][0] * temp_x[1] + rot_mat[0][1] * temp_y[1];
		temp_y[1] = rot_mat[1][0] * temp_x[1] + rot_mat[1][1] * temp_y[1];
		temp_x[1] = tmp;
		tmp = rot_mat[0][0] * temp_x[2] + rot_mat[0][1] * temp_y[2];
		temp_y[2] = rot_mat[1][0] * temp_x[2] + rot_mat[1][1] * temp_y[2];
		temp_x[2] = tmp;
		
		if(dx[1] - dx[0] == 0 && dy[1] - dy[0] == 0)
		{
			x[1] = (RATIO_TEST_STABLE*x[1] + temp_x[0]) / (RATIO_TEST_STABLE + 1);
			y[1] = (RATIO_TEST_STABLE*y[1] + temp_y[0]) / (RATIO_TEST_STABLE + 1);
		}
		else
		{
			x[1] = (RATIO_TEST_MOVE*x[1] + temp_x[0]) / (RATIO_TEST_MOVE + 1);
			y[1] = (RATIO_TEST_MOVE*y[1] + temp_y[0]) / (RATIO_TEST_MOVE + 1);
		}
		if(dx[2] - dx[0] == 0 && dy[2] - dy[0] == 0)
		{
			x[2] = (RATIO_TEST_STABLE*x[2] + temp_x[1]) / (RATIO_TEST_STABLE + 1);
			y[2] = (RATIO_TEST_STABLE*y[2] + temp_y[1]) / (RATIO_TEST_STABLE + 1);
		}
		else
		{
			x[2] = (RATIO_TEST_MOVE*x[2] + temp_x[1]) / (RATIO_TEST_MOVE + 1);
			y[2] = (RATIO_TEST_MOVE*y[2] + temp_y[1]) / (RATIO_TEST_MOVE + 1);
		}
		if(dx[3] - dx[0] == 0 && dy[3] - dy[0] == 0)
		{
			x[3] = (RATIO_TEST_STABLE*x[3] + temp_x[2]) / (RATIO_TEST_STABLE + 1);
			y[3] = (RATIO_TEST_STABLE*y[3] + temp_y[2]) / (RATIO_TEST_STABLE + 1);
		}
		else
		{
			x[3] = (RATIO_TEST_MOVE*x[3] + temp_x[2]) / (RATIO_TEST_MOVE + 1);
			y[3] = (RATIO_TEST_MOVE*y[3] + temp_y[2]) / (RATIO_TEST_MOVE + 1);
		}
		
		if(shape_setted)
		{
			rot_mat[0][0] = cos(angle_0*PI/180);
			rot_mat[0][1] = -sin(angle_0*PI/180);
			rot_mat[1][0] = sin(angle_0*PI/180);
			rot_mat[1][1] = cos(angle_0*PI/180);
			target_x[1] = rot_mat[0][0] * shape_x[1] + rot_mat[0][1] * shape_y[1];
			target_y[1] = rot_mat[1][0] * shape_x[1] + rot_mat[1][1] * shape_y[1];
			target_x[2] = rot_mat[0][0] * shape_x[2] + rot_mat[0][1] * shape_y[2];
			target_y[2] = rot_mat[1][0] * shape_x[2] + rot_mat[1][1] * shape_y[2];
			target_x[3] = rot_mat[0][0] * shape_x[3] + rot_mat[0][1] * shape_y[3];
			target_y[3] = rot_mat[1][0] * shape_x[3] + rot_mat[1][1] * shape_y[3];
		}
		else
		{			
			target_x[1] = x[1];
			target_y[1] = y[1];
			target_x[2] = x[2];
			target_y[2] = y[2];
			target_x[3] = x[3];
			target_y[3] = y[3];
		}
	}
}

void Send_Control_Cmd(void)
{
	int i = 0;
	rcv_displacement = 0;
	for(; i<13; i++)
	{
		USART_SendData(USART2, Control_Cmd_TX[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}
	Control_Cmd_TX[11] = 0;
}

void ParseData_Displacement(unsigned char data)
{
	static unsigned char ucRxBuffer[8];
	ucRxBuffer[ucRxCnt_dis++] = data;
	if(ucRxBuffer[0] != 0x7D)
		ucRxCnt_dis = 0;
	if(ucRxCnt_dis == 8)
	{
		if(ucRxBuffer[7] == 0x7C)
		{
			u8 i = 0;
			dx[MyID] = ucRxBuffer[1] << 8 | ucRxBuffer[2];
			dy[MyID] = ucRxBuffer[3] << 8 | ucRxBuffer[4];
			if(MyID == 0x00)
			{
				angle_0 = ucRxBuffer[5] << 8 | ucRxBuffer[6];
				if(angle_0 == 0)
					angle_0 = 360;
				else if(angle_0 == 360)
					angle_0 = 0;
			}
			for(; i<4 ;i++)
				Range_Info_TX[i + 5] = ucRxBuffer[i + 1];
			ucRxCnt_dis = 0;
			ucRxCnt_shape = 0;
			rcv_displacement = 1;
		}
		ucRxCnt_dis = 0;
	}
}

void ParseData_Shape(unsigned char data)
{
	static unsigned char ucRxBuffer[14];
	ucRxBuffer[ucRxCnt_shape++] = data;
	if(ucRxBuffer[0] != 0x7B)
		ucRxCnt_shape = 0;
	if(ucRxCnt_shape == 14)
	{
		if(ucRxBuffer[13] == 0x7A)
		{
			shape_x[1] = ucRxBuffer[1] << 8 | ucRxBuffer[2];
			shape_y[1] = ucRxBuffer[3] << 8 | ucRxBuffer[4];
			shape_x[2] = ucRxBuffer[5] << 8 | ucRxBuffer[6];
			shape_y[2] = ucRxBuffer[7] << 8 | ucRxBuffer[8];
			shape_x[3] = ucRxBuffer[9] << 8 | ucRxBuffer[10];
			shape_y[3] = ucRxBuffer[11] << 8 | ucRxBuffer[12];
			Control_Cmd_TX[11] = 1;
			shape_setted = 1;
			ucRxCnt_dis = 0;
		}
		ucRxCnt_shape = 0;
	}
}

void USART2_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
	{
		u8 tmp = USART_ReceiveData(USART2);
		ParseData_Displacement(tmp);
		ParseData_Shape(tmp);
	}
}

u8 Range_1_0(void)
{
	if(MyID == 0x00)
	{	
		u16 tmp = 0;
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_1_0;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x01, Cmd_Msg_TX, 4) == 0xFF);
		SetTick(COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x01, &tmp) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d01 += tmp;
	}
	else if(MyID == 0x01)
	{
		Delay(CMD_SEND_WAIT_TIME);
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x00, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Range_2_0(void)
{
	if(MyID == 0x00)
	{		
		u16 tmp = 0;
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_2_0;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x02, Cmd_Msg_TX, 4) == 0xFF);
		SetTick(COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x02, &tmp) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d02 += tmp;
	}
	else if(MyID == 0x02)
	{
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x00, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Range_3_0(void)
{
	if(MyID == 0x00)
	{		
		u16 tmp = 0;
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_3_0;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x03, Cmd_Msg_TX, 4) == 0xFF);
		SetTick(COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x03, &tmp) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d03 += tmp;
	}
	else if(MyID == 0x03)
	{
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x00, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Range_1_2(void)
{
	if(MyID == 0x00)
	{		
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_1_2;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x02, Cmd_Msg_TX, 4) == 0xFF);
		Delay(CMD_SEND_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Cmd_Msg_TX[0], 0x01, Cmd_Msg_TX, 4) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		SetTick(COMM_TIMEOUT + 2*COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receive_BrdMsg(Range_Info_RX[0], 0x02, Range_Info_RX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d12 += Range_Info_RX[3] << 8 | Range_Info_RX[4];
		dx[2] = Range_Info_RX[5] << 8 | Range_Info_RX[6];
		dy[2] = Range_Info_RX[7] << 8 | Range_Info_RX[8];
	}
	else if(MyID == 0x01)
	{
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x02, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	else if(MyID == 0x02)
	{
		SetTick(COMM_WAIT_TIME + 2*COMM_TIMEOUT + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x01, &d12) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		Range_Info_TX[3] = d12 >> 8;
		Range_Info_TX[4] = (u8) d12;
		Delay(COMM_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Range_Info_TX[0], 0x00, Range_Info_TX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Range_2_3(void)
{
	if(MyID == 0x00)
	{		
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_2_3;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x03, Cmd_Msg_TX, 4) == 0xFF);
		Delay(CMD_SEND_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Cmd_Msg_TX[0], 0x02, Cmd_Msg_TX, 4) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		SetTick(COMM_TIMEOUT + 2*COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receive_BrdMsg(Range_Info_RX[0], 0x03, Range_Info_RX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d23 += Range_Info_RX[3] << 8 | Range_Info_RX[4];
		dx[3] = Range_Info_RX[5] << 8 | Range_Info_RX[6];
		dy[3] = Range_Info_RX[7] << 8 | Range_Info_RX[8];
	}
	else if(MyID == 0x02)
	{
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x03, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	else if(MyID == 0x03)
	{
		SetTick(COMM_WAIT_TIME + 2*COMM_TIMEOUT + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x02, &d23) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		Range_Info_TX[3] = d23 >> 8;
		Range_Info_TX[4] = (u8) d23;
		Delay(COMM_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Range_Info_TX[0], 0x00, Range_Info_TX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Range_3_1(void)
{
	if(MyID == 0x00)
	{		
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = RANGE_CMD_3_1;
		while(Brd_Msg(Cmd_Msg_TX[0], 0x01, Cmd_Msg_TX, 4) == 0xFF);
		Delay(CMD_SEND_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Cmd_Msg_TX[0], 0x03, Cmd_Msg_TX, 4) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		SetTick(COMM_TIMEOUT + 2*COMM_WAIT_TIME + RANGING_TIMEOUT);
		while(Receive_BrdMsg(Range_Info_RX[0], 0x01, Range_Info_RX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		d13 += Range_Info_RX[3] << 8 | Range_Info_RX[4];
		dx[1] = Range_Info_RX[5] << 8 | Range_Info_RX[6];
		dy[1] = Range_Info_RX[7] << 8 | Range_Info_RX[8];
	}
	else if(MyID == 0x03)
	{
		Delay(COMM_WAIT_TIME);
		SetTick(RANGING_TIMEOUT);
		while(Initiator_Ranging(0x01, 1) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	else if(MyID == 0x01)
	{
		SetTick(COMM_WAIT_TIME + 2*COMM_TIMEOUT + RANGING_TIMEOUT);
		while(Receptor_Ranging(0x03, &d13) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		Range_Info_TX[3] = d13 >> 8;
		Range_Info_TX[4] = (u8) d13;
		Delay(COMM_WAIT_TIME);
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Range_Info_TX[0], 0x00, Range_Info_TX, 9) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	return 0x00;
}

u8 Brd_Coor(u8 TargetID)
{
	int i = 0;
	if(MyID == 0x00)
	{
		Delay(CMD_SEND_WAIT_TIME);
		Cmd_Msg_TX[3] = BRD_COOR_CMD;
		while(Brd_Msg(Cmd_Msg_TX[0], TargetID, Cmd_Msg_TX, 4) == 0xFF);
		Coordinate_Info_TX[3] = x[TargetID] >> 8;
		Coordinate_Info_TX[4] = (u8) x[TargetID];
		Coordinate_Info_TX[5] = y[TargetID] >> 8;
		Coordinate_Info_TX[6] = (u8) y[TargetID];
		Coordinate_Info_TX[7] = target_x[TargetID] >> 8;
		Coordinate_Info_TX[8] = (u8) target_x[TargetID];
		Coordinate_Info_TX[9] = target_y[TargetID] >> 8;
		Coordinate_Info_TX[10] = (u8) target_y[TargetID];
		Coordinate_Info_TX[11] = angle_0 >> 8;
		Coordinate_Info_TX[12] = (u8) angle_0;
		SetTick(COMM_TIMEOUT);
		while(Brd_Msg(Coordinate_Info_TX[0], TargetID, Coordinate_Info_TX, 13) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
	}
	else
	{
		SetTick(COMM_TIMEOUT);
		while(Receive_BrdMsg(Coordinate_Info_RX[0], 0x00, Coordinate_Info_RX, 13) == 0xFF)
		{
			if(GetTick() == 0)
				return 0xFF;
		}
		x[TargetID] = Coordinate_Info_RX[3] << 8 | Coordinate_Info_RX[4];
		y[TargetID] = Coordinate_Info_RX[5] << 8 | Coordinate_Info_RX[6];
		target_x[TargetID] = Coordinate_Info_RX[7] << 8 | Coordinate_Info_RX[8];
		target_y[TargetID] = Coordinate_Info_RX[9] << 8 | Coordinate_Info_RX[10];
		angle_0 = Coordinate_Info_RX[11] << 8 | Coordinate_Info_RX[12];
		for(; i<10; i++)
			Control_Cmd_TX[i + 1] = Coordinate_Info_RX[i + 3];
		Send_Control_Cmd();
		SetTick(USART_TIMEOUT);
		while(rcv_displacement == 0 && GetTick() != 0);
	}
	return 0x00;
}

u8 Run_Loop(void)
{
	success = 0;
	if(MyID == 0x00)
	{
		u8 i;
		d01 = d02 = d03 = d12 = d13 = d23 = 0;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_1_0() == 0xFF);
			i++;
		}
		d01 /= RANGING_TIME;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_2_0() == 0xFF);
			i++;
		}
		d02 /= RANGING_TIME;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_3_0() == 0xFF);
			i++;
		}
		d03 /= RANGING_TIME;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_1_2() == 0xFF);
			i++;
		}
		d12 /= RANGING_TIME;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_2_3() == 0xFF);
			i++;
		}
		d23 /= RANGING_TIME;
		i = 0;
		while(i < RANGING_TIME)
		{
			while(Range_3_1() == 0xFF);
			i++;
		}
		d13 /= RANGING_TIME;
		Send_Control_Cmd();
		SetTick(USART_TIMEOUT_0);
		while(rcv_displacement == 0 && GetTick() != 0);
		Calculate();
		while(Brd_Coor(0x01) == 0xFF);
		while(Brd_Coor(0x02) == 0xFF);
		while(Brd_Coor(0x03) == 0xFF);
	}
	else
	{
		while(Receive_BrdMsg(Cmd_Msg_RX[0], 0x00, Cmd_Msg_RX, 4) == 0xFF);
		switch(Cmd_Msg_RX[3])
		{
			case RANGE_CMD_1_0: Range_1_0(); break;
			case RANGE_CMD_2_0: Range_2_0(); break;
			case RANGE_CMD_3_0: Range_3_0(); break;
			case RANGE_CMD_1_2: Range_1_2(); break;
			case RANGE_CMD_2_3: Range_2_3(); break;
			case RANGE_CMD_3_1: Range_3_1(); break;
			case BRD_COOR_CMD: Brd_Coor(MyID); break;
			default: break;
		}
	}
	success = 1;
	return success;
}
