/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INITIATOR_H
#define __INITIATOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000


/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16473
#define RX_ANT_DLY 16473

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

u8 Initiator_Ranging(u8 TargetID, u8 force_timeout);
u8 Brd_Msg(u8 type, u8 TargetID, u8 *data, u8 length);


#endif /*__INITIATOR_H */
