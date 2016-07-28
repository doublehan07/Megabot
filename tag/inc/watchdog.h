#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
  
  
void WatchDog_Init(uint16_t timeout_ms);//init watchdog
void WatchDog_Feed(void);//feed dog

    
#ifdef __cplusplus
}
#endif
#endif