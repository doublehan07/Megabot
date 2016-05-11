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

//#include "mpi.h"
#include "stdio.h"
#include "ranging_api.h"
//typedef short u8;
//typedef unsigned int u16;
int dist[4][4];


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
    u8 rx_buffer[13];
    int id;
    int x,y;
    int dist[3];
    int channel;
}Node;


int Round(double m){
    return (int)(m+0.5);
}

void InitiatorMeasuring(u8* data, int len){
		Initiator_Ranging(data);
}
int ReceptorGrasping(Node* p, int *len, Node* source){
		Receptor_Listening(p->rx_buffer, len);
    return len;
}
int ReceptorMeasuring(Node* p, Node* source){//, int len){
		double dist = Receptor_Ranging(p->id);
    return dist; 
}

#endif /* Commtest_h */
