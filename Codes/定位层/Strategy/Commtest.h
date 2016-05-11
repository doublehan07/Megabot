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

#include "mpi.h"
#include "stdio.h"
typedef short u8;
typedef unsigned int u16;



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
    int pox,poy;
    int dist[4];
    int channel;
    int x_1,x_2;
    int y_1,y_2;
    int P1X, P1Y, P2X, P2Y, P3X, P3Y, P4X, P4Y;
}Node;

int Round(double m){
    return (int)(m+0.5);
}


void SendMsg(u8* data, int len){
    printf("len is %d in Sending data is %d,%d,%d,%d,%d\n", len,data[0],data[1],data[2],data[3],data[4]);
    int i = 0;
    for (i = 0; i<data[3] - data[2]+1; i++){
        if (data[2]+i != data[1])
        //printf("Sending data to id %d\n", data[2]+i);
            MPI_Send(data, len, MPI_SHORT, data[2]+i, data[2]+i, MPI_COMM_WORLD);
    }
}
void Initiator_Ranging(u8* data, int len){
    printf("len is %d in Sending data is %d,%d,%d,%d,%d, %d,%d,%d\n",len, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
    int i = 0;
    for (i = 0; i<data[3]-data[2]+1; ++i){
        if (data[2]+i != data[1])
            MPI_Send(data, len, MPI_SHORT, data[2]+i, data[2]+i, MPI_COMM_WORLD);
    }
}
int Receptor_Listening(Node* p, int len, Node* source){
    MPI_Status status;
    MPI_Recv(p->rx_buffer, len, MPI_SHORT, source->id, p->id, MPI_COMM_WORLD, &status);
            //printf("in id%d, data is %d%d%d%d%d\n",p->id, p->rx_buffer[0],p->rx_buffer[1],p->rx_buffer[2],p->rx_buffer[3],p->rx_buffer[4]);
    return len;
}
int Receptor_Ranging(Node* p, Node* source){//, int len){
    //int tag;
    //MPI_Status* status;
    //MPI_Recv(p.rx_buffer, len, MPI_SHORT_INT, source.id, p.id, MPI_COMM_WORLD, status);
    printf("source id %d, %d,%d\n", source->id,source->pox,source->poy);
    int dist = Round(sqrt((p->pox - source->pox)*(p->pox - source->pox)+(p->poy - source->poy)*(p->poy - source->poy)));
    return dist;
    
}

#endif /* Commtest_h */
