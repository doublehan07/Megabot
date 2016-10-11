//
//  Commtest.h
//  testMPI
//
//  Created by 温拓朴 on 16/5/9.
//  Copyright © 2016年 温拓朴. All rights reserved.
//

#ifndef Commtest_h
#define Commtest_h

#define P1 (0)
#define P2 (1)
#define P3 (2)
#define P4 (3)
#define Init 0x01
#define Finale 0x0F
#define fstMeasure 0x0A
#define secMeasure 0x0B
#define trdMeasure 0x0C
#define ChA 1
#define ChB 1

#include "stdio.h"
#include "ranging_api.h"
#include <math.h>

typedef struct Message{
    u8 Cmtype;
    u8 Sender;
    u8 RecieverL;
    u8 RecieverU;
    u16 coodiX;
    u16 coodiY;
    u16 Dist1;
    u16 Dist2;
    u8 CommCount;
}Message;

typedef struct Node{
    u8 rx_buffer[50];
    int id;
    int x,y;
    int dist[3];
    int channel;
}Node;


int Round(double);

void SGY_P1(void);
void SGY_P2(void);
void SGY_P3(void);
void SGY_P4(void);

#endif /* Commtest_h */
