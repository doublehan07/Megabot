

#include "watchdog.h"
#include "stm32f10x.h"

__IO uint32_t LsiFreq = 40000;

void WatchDog_Init(uint16_t timeout_ms)
{
  uint16_t reload;
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
                    Counter Reload Value = 250ms/IWDG counter clock period
                    = 250ms / (LSI/32)
                    = 0.25s * (LsiFreq/32)
                    = LsiFreq/(32 * 4)
                    = LsiFreq/128
  */
  
  reload = (uint16_t)((timeout_ms/1000.0)*(LsiFreq/32));
  IWDG_SetReload(reload);
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}

void WatchDog_Feed(void)
{
  IWDG_ReloadCounter();
}


