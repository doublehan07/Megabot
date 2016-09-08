/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STRATEGY_H
#define __STRATEGY_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/*
struct MyInfo
{
  present position
  present sangle
  my id
  corperation nodes[3]
  leader id[2] //fixed by upper controller
  my status{leader, nodes, triangle}
  my destination
}

struct FriendsInfo
{
  present position
  present sangle
  id
  status
  destination
}
*/
typedef struct
{
	double Rect_Axis[2];
	double Polar_Axis[2];
	u8 MyID;
}MyInfo;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STRATEGY_H */
