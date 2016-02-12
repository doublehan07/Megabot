#ifndef _PWM_H
#define _PWM_H
#include <stdint.h>
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h"
#include "timer.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

void pwm_set_DutyCycle1(uint16_t);
void pwm_config(uint16_t);
#endif
