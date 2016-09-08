#ifndef  DEVICEINFO
#define DEVICEINFO
#include "stm32f10x.h"
#define MASTERBOARD  0x01
#define ANCHORBOARD  0x02
#define TAGBOARD     0x03

#define MAC_ID   0xFFFF


//应有设备信息
typedef struct
{
  uint32_t MacID;
  uint32_t DataRate;
  uint8_t AnchorType;
  uint8_t ChannelNum;
} AnchorInfo;

typedef struct
{
  uint32_t MacID;
  uint32_t DataRate;
  uint8_t DeviceType;
  uint8_t ChannelNum;
} DeviceInfo;
#endif
