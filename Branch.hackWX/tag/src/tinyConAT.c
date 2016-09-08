#include "tinyConAT.h"
#include "port.h"

uint8_t AT_START[]="$$$";
uint8_t AT[]="AT";
uint8_t AT_WAP[]="AT+WAP";
uint8_t AT_WAUTO[]="AT+WAUTO";
uint8_t AT_DHCP[]="AT+DHCP";
uint8_t AT_IPSET[]="AT+IPSET";
uint8_t AT_NAUTO[]="AT+NAUTO";
uint8_t AT_W[]="AT&W";
uint8_t AT_RESET[]="AT+RESET";
uint8_t AT_OK[]="OK";
uint8_t AT_END[]={0x0d,0x0a};

uint8_t check_cmd_ok(uint8_t *cmd);

uint8_t config_tinycon_flag = 0;
uint8_t config_ok = 0;
void send_cmd(uint8_t *pBuf,uint32_t len)
{  
  if (pBuf == NULL)
  {
    return;
  }
 
  uint32_t i = 0;
  
  for (i = 0; i < len; i++)
  {
    USART_SendData(USART3, (uint8_t) pBuf[i]);  
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
  }
  
  //printf(pBuf);
}

uint8_t set_tinycon_baud(uint32_t baud)
{
  uint8_t cmd[64] = {0}; 
  config_tinycon_flag = 1;
  Delay(100);
  send_cmd(AT_START,strlen((const char*)AT_START));  
  Delay(100);
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"ATB=%d,8,n,1,nfc\r\n",baud);
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  check_cmd_ok(cmd);
 
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT&W\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  check_cmd_ok(cmd);
  config_tinycon_flag = 0;
  return 0;
}

uint8_t check_cmd_ok(uint8_t *cmd)
{
  uint32  iTick =  portGetTickCount();
  while(1)
  {
    if(config_ok == 1)
    {
      config_ok = 0;
      printf("config %s ok \r\n",cmd);
      return 0;
    }
    if(iTick+5 <= portGetTickCount())
    {
      printf("config %s fail \r\n",cmd);
      return 1;
    }
  }
}

uint8_t set_tinycon_serverinfo()
{
  uint8_t cmd[64] = {0}; 
  config_tinycon_flag = 1;
  Delay(100);
  send_cmd(AT_START,strlen((const char*)AT_START));  
  Delay(100);
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+NAUTO=1,0,192.168.1.100,44333,44332\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  check_cmd_ok(cmd);
 
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT&W\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  check_cmd_ok(cmd);
  
   memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+RESET\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  check_cmd_ok(cmd);
  config_tinycon_flag = 0;
  return 0;
}


uint8_t start_ap_mode()
{
  uint8_t cmd[64] = {0};
  
  config_tinycon_flag = 1;
  Delay(100);
  send_cmd(AT_START,strlen((const char*)AT_START));
  Delay(3000);
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"ATS2=1\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+WAUTO=1,woxu_wirelss,6,12345678\r\n");
  
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"at+dhcpsrvr=1\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+IPSET=192.168.1.120,255.255.255.0,192.168.1.100\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+NAUTO=1,0,192.168.1.100,44333,44332\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT&W\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+RESET\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  config_tinycon_flag = 0;
  
  return 0;
}

uint8_t start_station_mode()
{
  uint8_t cmd[64] = {0};
  
  config_tinycon_flag = 1;
  Delay(100);
  send_cmd(AT_START,strlen((const char*)AT_START));
  Delay(3000);
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"ATS2=1\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+WAUTO=0,woxu_wireless,12345678\r\n");
  
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+DHCP=0\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+IPSET=192.168.1.120,255.255.255.0,192.168.1.100\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+NAUTO=1,0,192.168.1.100,44333,44332\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT&W\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  memset(cmd,0,sizeof(cmd));
  sprintf((char*)cmd,"AT+RESET\r\n");
  send_cmd(cmd,strlen((const char*)cmd));
  Delay(100);
  if (0 != check_cmd_ok(cmd))
  {
    return 1;
  }
  
  config_tinycon_flag = 0;
  
  return 0;
}

