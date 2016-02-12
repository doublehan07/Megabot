#include "common.h"



/*****************************************************
函数: void Delay(vu32 nCount)
功能: 延时指定时间
参数: vu32 nCount 延时指定时间
返回: 无
******************************************************/
void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}
