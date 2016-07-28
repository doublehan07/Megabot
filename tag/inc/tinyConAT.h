/**
  ******************************************************************************
  * @file    tinyConAT.h
  * @author  WoXu Team
  * @version V1.0
  * @date    31-08-2015
  * @brief   AT cmd for tinyCon 
  ******************************************************************************
  */
#ifndef __TINYCON_AT_H_
#define __TINYCON_AT_H_ 
#include <stdint.h>

typedef enum _WIFI_MODE
{
	STATION = 0,
    AP      = 1
}WIFI_MODE;

typedef struct UWB_Data_Array
{
  uint8_t	size;
  uint8_t	buf[64];
}UWB_Data_Array;

uint8_t start_ap_mode();
uint8_t start_station_mode();
void send_cmd(uint8_t *pBuf,uint32_t len);
uint8_t set_tinycon_baud(uint32_t baud);
uint8_t set_tinycon_serverinfo();
#endif   //__TINYCON_AT_H_