//
//  Commtest.c
//  testMPI
//
//  Created by 温拓朴 on 16/5/9.
//  Copyright © 2016年 温拓朴. All rights reserved.
//
#define COMM
#ifdef COMM


#include "Commtest.h"

Node p[4];
Node Recp[5];
//u8 buffer[100];


u8 rx_buffer[30];
int Done[2];
int n=4;
void SendMsg(u8*, int);
void Initiator_Ranging(u8*, int);
int Receptor_Listening(Node* p, int len, Node* source);
int Receptor_Ranging(Node* p, Node* source);

u8* SendMessage(Message msg, int len){
    if (len == 13){
        u8 data[13];
        data[0] = msg.Cmtype;
        data[1] = msg.Sender;
        data[2] = msg.RecieverL;
        data[3] = msg.RecieverU;
        data[4] = msg.coodiX >> 8;
        data[5] = msg.coodiX & 0xFF;
        data[6] = msg.coodiY >> 8;
        data[7] = msg.coodiY & 0xFF;
        data[8] = msg.Dist1 >> 8;
        data[9] = msg.Dist1 & 0xFF;
        data[10] = msg.Dist2 >> 8;
        data[11] = msg.Dist2 & 0xFF;
        data[12] = msg.CommCount;
        if (msg.CommCount == 0){
            //printf("%d,%d,%d", Rounddata[0],Rounddata[1],Rounddata[12]);
            SendMsg(data, len);
            //int tag = 0;
            //MPI_Status status;
            //MPI_Recv(data, len, MPI_SHORT, data[1], tag, MPI_COMM_WORLD, &status)
        }
        else{
            //printf("%d,%d,%d", data[0],data[1],data[12]);
            Initiator_Ranging(data, len);
        }
        return data;
    }
    else{
        u8 data[5];
        data[0] = msg.Cmtype;
        data[1] = msg.Sender;
        data[2] = msg.RecieverL;
        data[3] = msg.RecieverU;
        data[4] = msg.CommCount;
        
        SendMsg(data, 5);
        return data;
    }
}

Message SaveMessage(u8* data, int len){
    Message msg;
    if (len == 13){
        msg.Cmtype = data[0];
        msg.Sender = data[1];
        msg.RecieverL = data[2];
        msg.RecieverU = data[3];
        msg.coodiX = ((data[4] & 0xFF)<< 8) | (data[5] & 0xFF);
        msg.coodiY = ((data[6] & 0xFF)<< 8) | (data[7] & 0xFF);
        msg.Dist1 = ((data[8] & 0xFF)<< 8) | (data[9] & 0xFF);
        msg.Dist2 = ((data[10] & 0xFF)<< 8) | (data[11] & 0xFF);
        msg.CommCount = data[12];
    }
    else{
        msg.Cmtype = data[0];
        msg.Sender = data[1];
        msg.RecieverL = data[2];
        msg.RecieverU = data[3];
        msg.CommCount = data[4];
    }
    return msg;
}

void SGY_P1(){
    //P1's strategy
    p[0].id = P1;
    p[0].x = 0;
    p[0].y = 0;
    
    
    Message upperAsk;
    Message msg;
    int len;
    int flag = 1;
    int dist1[3];
    int dist2[3];
    int dist3[3];
    int dist4[3];
    int dist[6];
    //upperAsk = SaveMessage(p[0].rx_buffer, len);
    if (flag == 1){
        msg.Cmtype = Init;
        msg.Sender = P1;
        msg.RecieverL = P2;
        msg.RecieverU = P4;
        msg.CommCount = 0;
        SendMessage(msg, 5);
    }
    
    Receptor_Listening(&p[0],5,&p[3]);
    upperAsk = SaveMessage(p[0].rx_buffer, 5);
    if (upperAsk.Cmtype == Init){
        if (upperAsk.Sender == P4){
            //start the four point locating...
            msg.Cmtype = fstMeasure;
            msg.Sender = P1;
            msg.RecieverL = P1;
            msg.RecieverU = P4;
            msg.coodiX = 0;
            msg.coodiY = 0;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            printf("P1 has Initialed in id%d\n", p[0].id);
            SendMessage(msg,13);//Measuring P1 with P2 P3 P4
        }
    }
    
    Receptor_Listening(&p[0],13,&p[1]);
    upperAsk = SaveMessage(p[0].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        dist1[0] = Receptor_Ranging(&p[0],&p[1]);
        //get the position of P2
    }
    
    Receptor_Listening(&p[0],13,&p[2]);
    upperAsk = SaveMessage(p[0].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        dist1[1] = Receptor_Ranging(&p[0],&p[2]);
        //get the position of P2
    }
    
    Receptor_Listening(&p[0],13,&p[3]);
    upperAsk = SaveMessage(p[0].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        dist1[2] = Receptor_Ranging(&p[0],&p[3]);
        msg.Cmtype = fstMeasure;
        msg.Sender = P1;
        msg.RecieverL = P4;
        msg.RecieverU = P4;
        msg.coodiX = dist1[0];
        msg.coodiY = dist1[1];
        msg.Dist1 = dist1[2];
        msg.Dist2 = 0;
        msg.CommCount = 0;
        SendMessage(msg, 13);
        //get the position of P1
    }
    
    //Receptor_Listening(p[0], 5, Recp[n/2 -1]);
    //upperAsk = SaveMessage(p[0].rx_buffer, 5);
    if (upperAsk.Cmtype == secMeasure){
        //P1 measuring with 1:n/2
        if (upperAsk.Sender == n/2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P1 && upperAsk.Cmtype == 0){
            msg.Cmtype = secMeasure;
            msg.Sender = P1;
            msg.RecieverL = 1;
            msg.RecieverU = n/2;
            msg.coodiX = p[0].x;
            msg.coodiY = p[0].y;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            //SendMessage(msg,13);
        }
    }
    printf("p1's Position is %d, %d, %d", dist1[0], dist1[1],dist1[2]);
    
    // if (upperAsk.Cmtype] = endMeasure){
    // 	if (msg.Sender] == 1 && msg.RecieverL] == P1 && msg.RecieverU] == P1){
    
    // 	}
    // }
}

void SGY_P2(){
    Message msg,upperAsk;
    p[1].id = P2;
    p[1].x = 0;
    p[1].y = 0;
    p[1].pox = 100;
    p[1].poy = 0;
    Done[0] = 0;
    Done[1] = 0;

    
    Receptor_Listening(&p[1], 5, &p[0]);
    upperAsk = SaveMessage(p[1].rx_buffer, 5);
    //printf("in id%d, data is %d%d%d%d%d\n",p[1].id, rx_buffer[0],rx_buffer[0],rx_buffer[0],rx_buffer[0],rx_buffer[0]);
    if (upperAsk.Cmtype == Init){
        //printf("P2 has Initialed\n");
    }
    
    Receptor_Listening(&p[1], 13, &p[0]);
    upperAsk = SaveMessage(p[1].rx_buffer , 13);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
            p[1].dist[0] = Receptor_Ranging(&p[1],&p[0]);
            
            //get the position of P2
        }
    
    Receptor_Listening(&p[1] , 5, &p[3]);
    upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){//P4 replies Measuring with P1, then tell P2 to start measuring P3, P4
            //then P2 start fstMeasure with P3 P4
            msg.Cmtype = fstMeasure;
            msg.Sender = P2;
            msg.RecieverL = P1;
            msg.RecieverU = P4;
            msg.coodiX = 0;
            msg.coodiY = 0;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            SendMessage(msg,13);
        }
    
    Receptor_Listening(&p[1], 13, &p[2]);
    upperAsk = SaveMessage(p[1].rx_buffer , 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P3 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        p[1].dist[1] = Receptor_Ranging(&p[1],&p[2]);
        
        //get the position of P2
    }
    
    Receptor_Listening(&p[1], 13, &p[3]);
    upperAsk = SaveMessage(p[1].rx_buffer , 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P4 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        p[1].dist[2] = Receptor_Ranging(&p[1],&p[3]);
        //delay(2)
        msg.Cmtype = fstMeasure;
        msg.Sender = P2;
        msg.RecieverL = P4;
        msg.RecieverU = P4;
        msg.coodiX = p[1].dist[0];
        msg.coodiY = p[1].dist[1];
        msg.Dist1 = p[1].dist[2];
        msg.Dist2 = 0;
        msg.CommCount = 0;
        SendMessage(msg, 13);
        
        //get the position of P2
    }
    
    Receptor_Listening(&p[1], 13, &p[3]);
    upperAsk = SaveMessage(p[1].rx_buffer , 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){
        p[1].x = upperAsk.coodiX;
        p[1].y = upperAsk.coodiY;
        //get the result of P4;
    }
    
    //Receptor_Listening(p[1].rx_buffer, 5, p[3]);
    //upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//P4 replies Measuring with P3, then tell P2 and P3 to start the secondstep
            //then P2 start secMeasure with 1:n/2
            msg.Cmtype = secMeasure;
            msg.Sender = P2;
            msg.RecieverL = 1;
            msg.RecieverU = n/2;
            msg.coodiX = p[1].x;
            msg.coodiY = p[1].y;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            //SendMessage(msg,13);
        }
    
    
    //Receptor_Listening(p[1] , 5, Recp[n/2 -1]);
    //upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == secMeasure && upperAsk.Sender == n/2 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){
            Done[0] = 1;//the measuring with 1:n/2 is done
        }
    
    //Receptor_Listening(p[1].rx_buffer, 5, Recp[n-1]);
    //upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == secMeasure && upperAsk.Sender == n && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){
            Done[1] = 1;//the measuring with n/2+1:n is done
            if (Done[0] == 1 && Done[1] == 1){
                msg.Cmtype = trdMeasure;
                msg.Sender = P2;
                msg.RecieverL = 1;
                msg.RecieverU = n;
                msg.CommCount = 0;
                //start the trdMeasure
                //SendMessage(msg,5);
            }
        }
    printf("p2's Position is %d, %d", p[1].x, p[1].y);
}

void SGY_P3(){
    p[2].id = P3;
    p[2].channel = ChA;
    p[2].x = 0;
    p[2].y = 0;
    p[2].pox = 50;
    p[2].poy = 400;
    Message msg, upperAsk;
    int dist1[3];
    int dist2[3];
    int dist3[3];
    int dist4[3];
    int dist[6];
    
    Receptor_Listening(&p[2], 5, &p[0]);
    upperAsk = SaveMessage(p[2].rx_buffer, 5);
        //printf("in id%d, data is %d%d%d%d%d\n",p[1].id, rx_buffer[0],rx_buffer[0],rx_buffer[0],rx_buffer[0],rx_buffer[0]);
    if (upperAsk.Cmtype == Init){
        //printf("P2 has Initialed\n");
    }
    
    Receptor_Listening(&p[2] , 13, &p[0]);
    upperAsk = SaveMessage(p[2].rx_buffer, 13);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[2].dist[0] = Receptor_Ranging(&p[2], &p[0]);
        }//measuring P1 with P2 P3 P4
    
    Receptor_Listening(&p[2] , 13, &p[1]);
    upperAsk = SaveMessage(p[2].rx_buffer, 13);
    //mesuring P2 with P3 P4
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[2].dist[1] = Receptor_Ranging(&p[2], &p[1]);
//            double pp, S;
//            if (p[2].dist[0] != 0xFFFF && p[2].dist[1] != 0xFFFF){
//                pp = 1.0*(p[2].dist[1] + p[2].dist[0] + upperAsk.coodiX) / 2.0;
//                S = sqrt(pp*(pp-p[2].dist[1])*(pp-p[2].dist[0])*(pp-upperAsk.coodiX));
//                
//                p[2].y = Round(2 * S/upperAsk.coodiX);
//                p[2].x = Round(sqrt(p[2].dist[0] * p[2].dist[0] - p[2].y * p[2].y));
//            }
            // Calculate P3's position
        }
    
    Receptor_Listening(&p[2] , 5, &p[3]);
    upperAsk = SaveMessage(p[2].rx_buffer, 5);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P3 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//P2 has measured P3 P4, then P4 should tell P3 to start measuring P3 P4
            msg.Cmtype = fstMeasure;
            msg.Sender = P3;
            msg.RecieverL = P1;
            msg.RecieverU = P4;
            msg.coodiX = 0;
            msg.coodiY = 0;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            SendMessage(msg,13);//start measuring
        }
    
    //P4's measuring
    Receptor_Listening(&p[2] , 13, &p[3]);
    upperAsk = SaveMessage(p[2].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
        p[2].dist[2] = Receptor_Ranging(&p[2], &p[3]);
        //delay(3);
        msg.Cmtype = fstMeasure;
        msg.Sender = P3;
        msg.RecieverL = P4;
        msg.RecieverU = P4;
        msg.coodiX = p[2].dist[0];
        msg.coodiY = p[2].dist[1];
        msg.Dist1 = p[2].dist[2];
        msg.Dist2 = 0;
        msg.CommCount = 0;
        SendMessage(msg,13);//start measuring
        
        
    }//measuring P4 with P2 P3 P1
    
    Receptor_Listening(&p[2], 13, &p[3]);
    upperAsk = SaveMessage(p[2].rx_buffer , 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){
        p[2].x = upperAsk.Dist1;
        p[2].y = upperAsk.Dist2;
        //get the result of P4;
    }
    
    //Receptor_Listening(p[1].rx_buffer, 5, p[3]);
    //upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == secMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//fstMeasuring is done then P3 should start the seconds step
            p[2].channel = ChB;
            msg.Cmtype = secMeasure;
            msg.Sender = P3;
            msg.RecieverL = n/2+1;
            msg.RecieverU = n;
            msg.coodiX = p[2].x;
            msg.coodiY = p[2].y;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            //SendMessage(msg,13);
        }
    printf("p3's Position is %d, %d", p[2].x, p[2].y);
}

void SGY_P4(){
    p[3].id = P4;
    p[3].channel = ChA;
    p[3].x = 0;
    p[3].y = 0;
    p[3].pox = 500;
    p[3].poy = 480;
    Message msg, upperAsk;
    int dist[4][4];
    int i;
    
    Receptor_Listening(&p[3],5,&p[0]);
    upperAsk = SaveMessage(p[3].rx_buffer, 5);

    if (upperAsk.Cmtype == Init){
        msg.Cmtype = Init;
        msg.Sender = P4;
        msg.RecieverL = P1;
        msg.RecieverU = P1;
        msg.CommCount = 0;
        //printf("P4 has Initialed\n");
        SendMessage(msg, 5);
    }
    
    
    Receptor_Listening(&p[3], 13, &p[0]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[3].dist[0] = Receptor_Ranging(&p[3],&p[0]);
            msg.Cmtype = fstMeasure;
            msg.Sender = P4;
            msg.RecieverL = P2;
            msg.RecieverU = P2;
            msg.CommCount = 0;
            printf("id 4 dist 1 = %d\n", p[3].dist[0]);
            SendMessage(msg,5);//tell P2 to Start
        }
    
    Receptor_Listening(&p[3], 13, &p[1]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[3].P2X = upperAsk.coodiX;
            printf("p[3].P2X is %d\n", p[3].P2X);
            p[3].dist[1] = Receptor_Ranging(&p[3],&p[1]);
            msg.Cmtype = fstMeasure;
            msg.Sender = P4;
            msg.RecieverL = P3;
            msg.RecieverU = P3;
            msg.CommCount = 0;
            printf("id 4 dist 2 = %d\n", p[3].dist[1]);
            SendMessage(msg,5);//tell P3 to Start
        }
    
    Receptor_Listening(&p[3] , 13, &p[2]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[3].dist[2] = Receptor_Ranging(&p[3],&p[2]);
            msg.Cmtype = fstMeasure;
            msg.Sender = P4;
            msg.RecieverL = P1;
            msg.RecieverU = P4;
            msg.coodiX = 0;
            msg.coodiY = 0;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            SendMessage(msg, 13);
        }
    
    //get result from P1;
    Receptor_Listening(&p[3] , 13, &p[0]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[0][1] = upperAsk.coodiX;//P12
        dist[0][2] = upperAsk.coodiY;//P13
        dist[0][3] = upperAsk.Dist1;//P14
    }
    
    //get result from P2
    Receptor_Listening(&p[3] , 13, &p[1]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[1][0] = upperAsk.coodiX;//P21
        dist[1][2] = upperAsk.coodiY;//P23
        dist[1][3] = upperAsk.Dist1;//P24
    }
    
    //get result from P3
    Receptor_Listening(&p[3] , 13, &p[2]);
    upperAsk = SaveMessage(p[3].rx_buffer, 13);
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[2][0] = upperAsk.coodiX;//P31
        dist[2][1] = upperAsk.coodiY;//P32
        dist[2][3] = upperAsk.Dist1;//P34
    }
    //printf("p4's Position is %d, %d, %d\n", p[3].dist[0], p[3].dist[1], p[3].dist[2]);
    //printf("%d,%d,%d,%d,%d,%d\n", dist[0][0],dist[0][1],dist[0][2],dist[1][0],dist[1][1],dist[1][2]);
    //calculate!
    double x1 = 0, y1 = 0, y2 = 0;
    double x2,x3,x4,y3,y4,tx1,tx2,tx3,tx4,ty1,ty2,ty3,ty4,dist_12,dist_13,dist_14,dist_23,dist_24,dist_34;
    double pp,S,a,b,c,d,b1,b2,b3,B1,B2,delta,rate;
    dist[3][0]=p[3].dist[0];
    dist[3][1]=p[3].dist[1];
    dist[3][2]=p[3].dist[2];
    dist[0][1]=(dist[0][1]+dist[1][0])*0.5;//P12
    dist[0][2]=(dist[0][2]+dist[2][0])*0.5;//P13
    dist[0][3]=(dist[0][3]+dist[3][0])*0.5;//P14
    dist[1][2]=(dist[1][2]+dist[2][1])*0.5;//P23
    dist[1][3]=(dist[1][3]+dist[3][1])*0.5;//P24
    dist[2][3]=(dist[2][3]+dist[3][2])*0.5;//P34
    
    x2=dist[0][1];
    pp = 1.0*(dist[1][2] + dist[0][2] + dist[0][1]) / 2.0;
    S = sqrt(pp*(pp-dist[1][2])*(pp-dist[0][2])*(pp-dist[0][1]));
    
    y3 = Round(2 * S/dist[0][1]);
    x3 = Round(sqrt(dist[0][2] * dist[0][2] - 2 * S/dist[0][1] * 2 * S/dist[0][1]));
    
    a = (x2-x1)*(x2-x1)+(x3-x2)*(x3-x2)+(x1-x3)*(x1-x3);
    d = (y2-y1)*(y2-y1)+(y3-y2)*(y3-y2)+(y1-y3)*(y1-y3);
    b = (x2-x1)*(y2-y1)+(x3-x2)*(y3-y2)+(x1-x3)*(y1-y3);
    c = b;
    delta = a*d-b*c;
    b1 = 0.5*(dist[0][3]*dist[0][3] - dist[1][3]*dist[1][3] + (x2-x1)*(x2-x1)+ (y2-y1)*(y2-y1));
    b2 = 0.5*(dist[1][3]*dist[1][3] - dist[2][3]*dist[2][3] + (x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
    b3 = 0.5*(dist[2][3]*dist[2][3] - dist[0][3]*dist[0][3] + (x1-x3)*(x1-x3)+ (y1-y3)*(y1-y3));
    B1 = (x2-x1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(x3-x2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(x1-x3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
    B2 = (y2-y1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(y3-y2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(y1-y3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
    x4 = Round((d * B1 - b * B2) / delta);
    y4 = Round((-c * B1 + a * B2) / delta);
    
    rate = 0.5;
    
    for (i=0; i < 10; i++){
        dist_12=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
        dist_13=sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1));
        dist_14=sqrt((x4-x1)*(x4-x1)+(y4-y1)*(y4-y1));
        dist_23=sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3));
        dist_24=sqrt((x2-x4)*(x2-x4)+(y2-y4)*(y2-y4));
        dist_34=sqrt((x3-x4)*(x3-x4)+(y3-y4)*(y3-y4));
        tx2 = x2 - rate *((x2-x1)*(1-dist[0][1]/dist_12)+(x2-x3)*(1-dist[1][2]/dist_23)+(x2-x4)*(1-dist[1][3]/dist_24));
        tx3 = x3 - rate *((x3-x1)*(1-dist[0][2]/dist_13)+(x3-x2)*(1-dist[1][2]/dist_23)+(x3-x4)*(1-dist[2][3]/dist_34));
        tx4 = x4 - rate *((x4-x1)*(1-dist[0][3]/dist_14)+(x4-x2)*(1-dist[1][3]/dist_24)+(x4-x3)*(1-dist[2][3]/dist_34));
        ty3 = y3 - rate *((y3-y1)*(1-dist[0][2]/dist_13)+(y3-y2)*(1-dist[1][2]/dist_23)+(y3-y4)*(1-dist[2][3]/dist_34));
        ty4 = y4 - rate *((y4-y1)*(1-dist[0][3]/dist_14)+(y4-y2)*(1-dist[1][3]/dist_24)+(y4-y3)*(1-dist[2][3]/dist_34));
        x2 = tx2;
        x3 = tx3;
        x4 = tx4;
        y3 = ty3;
        y4 = ty4;
    }
    
    
    p[3].x = x4;
    p[3].y = y4;
    msg.Cmtype = fstMeasure;
    msg.Sender = P4;
    msg.RecieverL = P2;
    msg.RecieverU = P3;
    msg.coodiX = x2;
    msg.coodiY = y2;
    msg.Dist1 = x3;
    msg.Dist2 = y3;
    msg.CommCount = 0;
    SendMessage(msg, 13);
    

    
//            if (p[3].dist[2] != 0xFFFF && p[3].dist[1] != 0xFFFF && p[3].dist[0] != 0xFFFF){
//                double pp, S, tmpy,a,b,c,d, x1,x2,x3,y1,y2,y3,delta,b1,b2,b3,B1,B2;
//                //pp = 1.0*(p[3].dist[1] + p[3].dist[0] + p[3].P2X) / 2.0;
//                //printf("pp is %lf",pp);
//                
//                
//                
//                //this is a calculation in given x1,y1, x2,y2, x3,y3, and dist[0],dist[1],dist[2] and dist[3]=P2X, dist[4]=
//                x1=y1=0;
//                x2=p[3].P2X,y2=0;
//                x3=upperAsk.coodiX,y3=upperAsk.coodiY;
//                printf("%lf\n", x2);
//                a = (x2-x1)*(x2-x1)+(x3-x2)*(x3-x2)+(x1-x3)*(x1-x3);
//                d = (y2-y1)*(y2-y1)+(y3-y2)*(y3-y2)+(y1-y3)*(y1-y3);
//                b = (x2-x1)*(y2-y1)+(x3-x2)*(y3-y2)+(x1-x3)*(y1-y3);
//                c = b;// = (x2-x1)*(y2-y1)+(x3-x2)*(y3-y2)+(x1-x3)*(y1-y3);
//                delta = a*d-b*c;
//                b1 = 0.5*(p[3].dist[0]*p[3].dist[0] - p[3].dist[1]*p[3].dist[1] + (x2-x1)*(x2-x1)+ (y2-y1)*(y2-y1));
//                b2 = 0.5*(p[3].dist[1]*p[3].dist[1] - p[3].dist[2]*p[3].dist[2] + (x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2));
//                b3 = 0.5*(p[3].dist[2]*p[3].dist[2] - p[3].dist[0]*p[3].dist[0] + (x1-x3)*(x1-x3)+ (y1-y3)*(y1-y3));
//                B1 = (x2-x1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(x3-x2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(x1-x3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
//                B2 = (y2-y1)*(b1+x1*x2+y1*y2-x1*x1-y1*y1)+(y3-y2)*(b2+x2*x3+y2*y3-x2*x2-y2*y2)+(y1-y3)*(b3+x1*x3+y1*y3-x3*x3-y3*y3);
//                p[3].x = Round((d * B1 - b * B2) / delta);
//                p[3].y = Round((-c * B1 + a * B2) / delta);
//                
//                
////                S = sqrt(pp*(pp-p[3].dist[1])*(pp-p[3].dist[0])*(pp-p[3].P2X));
////                tmpy =2 * S / p[3].P2X;
////                p[3].y = Round(tmpy);
////                p[3].x = Round(sqrt(p[3].dist[0] * p[3].dist[0] - tmpy * tmpy));
////                if ((p[3].x-upperAsk.coodiX)*(p[3].x-upperAsk.coodiX)+(p[3].y-upperAsk.coodiY)*(p[3].x-upperAsk.coodiY) - p[3].dist[3]*p[3].dist[3] >
////                    (p[3].x-upperAsk.coodiX)*(p[3].x-upperAsk.coodiX)+(p[3].y+upperAsk.coodiY)*(p[3].x+upperAsk.coodiY) - p[3].dist[3]*p[3].dist[3]){
////                    p[3].y = -p[3].y;
//                    // Calculate P4's position
//            
//                //fstMeasure completed, send message to let P2 and P3 to start secMeasure
//                msg.Cmtype = secMeasure;
//                msg.Sender = P4;
//                msg.RecieverL = P2;
//                msg.RecieverU = P3;
//                msg.coodiX = 0;
//                msg.coodiY = 0;
//                msg.Dist1 = 0;
//                msg.Dist2 = 0;
//                msg.CommCount = 0;
//                //SendMessage(msg,13);
//                p[3].channel = ChB;
//            }
    
    
    
//    Receptor_Listening(p[1].rx_buffer, 5, p[3]);
//    upperAsk = SaveMessage(p[1].rx_buffer, 5);
        if (upperAsk.Cmtype == secMeasure && upperAsk.Sender == n && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
            //secMeasure of P3 is done, it's turn to P4 to measuring n/2+1:n;
            msg.Cmtype = secMeasure;
            msg.Sender = P4;
            msg.RecieverL = n/2+1;
            msg.RecieverU = n;
            msg.coodiX = p[3].x;
            msg.coodiY = p[3].y;
            msg.Dist1 = 0;
            msg.Dist2 = 0;
            msg.CommCount = 1;
            //SendMessage(msg,13);
        }
    printf("P4's position is %d, %d\n", p[3].x, p[3].y);
}

/*void SGY_Receptor(){
    //variances list
    Message msg,upperAsk;
    int len;
    while ((len = Receptor_Listening(0)) != 0);
    upperAsk = SaveMessage(rx_buffer, len);
    //Listening...
    //Receptor_Listening()
    if (upperAsk.Cmtype == Init){
        P1X = 0;
        P2X = 0;
        P3X = 0;
        P4X = 0;
        x = 0;
        y = 0;
        x_1 = 0;
        x_2 = 0;
        y_1 = 0;
        y_2 = 0;
        if (id == n){
            msg.Cmtype = Init;
            msg.Sender = n;
            msg.RecieverL = P1;
            msg.RecieverU = P1;
            msg.CommCount = 0;
            SendMessage(msg,5);
        }
        //tell Leader Init is done
        
        if (id > n/2){
            channel = ChB;
        }
        else{
            channel = ChA;
        }
    }
    
    if (upperAsk.Cmtype == secMeasure){												 //==n/2+1 //==n
        if (upperAsk.Sender == P3 && upperAsk.RecieverL <= id && upperAsk.RecieverU >= id && upperAsk.CommCount == 1){//P3 measuring with n/2+1 : n
            P3X = upperAsk.coodiX;
            P3Y = upperAsk.coodiY;
            dist[0] = Receptor_Ranging((id-1) % (n/2));
            if (id == n && dist[0] != 0xFFFF){//P3 done, let P4 start second measuring
                msg.Cmtype = secMeasure;
                msg.Sender = n;
                msg.RecieverL = P4;
                msg.RecieverU = P4;
                msg.CommCount = 0;
                SendMessage(msg,5);
            }
        }												//==n/2+1 						//==n
        if (upperAsk.Sender == P4 && upperAsk.RecieverL <= id  && upperAsk.RecieverU >= id && upperAsk.CommCount == 1){//P4 measure with n/2+1 : n
            P4X = upperAsk.coodiX;
            P4Y = upperAsk.coodiY;
            dist[1] = Receptor_Ranging((id-1) % (n/2));
            if (dist[1] != 0xFFFF){
                if (id >= n/2 + 1){// if this measuring is done, n/2+2:n should reverse channel to ChA
                    channel = ChA;
                }
                if (id == n){
                    msg.Cmtype = secMeasure;
                    msg.Sender = n;
                    msg.RecieverL = P2;
                    msg.RecieverU = P2;
                    msg.CommCount = 0;
                    SendMessage(msg,5);
                }
                double d, p, S, h, a, x_, y_;
                d = sqrt((P3X - P4X)*(P3X - P4X) + (P3Y - P4Y)*(P3Y - P4Y));
                p = (dist[0] + dist[1] + d) / 2;
                S = sqrt(p*(p-dist[0])*(p-dist[1])*(p-d));
                h = 2.0 * S / d;
                a = sqrt(dist[0]*dist[0] - h * h);
                x_ = a*1.0 / d * P2X + (1 - a*1.0 / d) * P3X;
                y_ = a*1.0 / d * P2Y + (1 - a*1.0 / d) * P3Y;
                x_1 = x_ + (P4Y - P3Y) * h / d;
                y_1 = y_ - (P4X - P3X) * h / d;
                x_2 = x_ - (P4Y - P3Y) * h / d;
                y_2 = y_ + (P4X - P3X) * h / d;
            }
        }
        
        if (upperAsk.Sender == P2 && upperAsk.RecieverL <= id && upperAsk.RecieverU >= id && upperAsk.CommCount == 1){
            P2X = upperAsk.coodiX;
            P2Y = upperAsk.coodiY;
            dist[1] = Receptor_Ranging((id - 1) % (n / 2));
            if (id == n/2 && dist[1] != 0xFFFF){
                msg.Cmtype = secMeasure;
                msg.Sender = n/2;
                msg.RecieverL = P1;
                msg.RecieverU = P1;
                msg.CommCount = 0;
                SendMessage(msg,5);
            }
        }
        
        if (upperAsk.Sender == P1 && upperAsk.RecieverL <= id && upperAsk.RecieverU >= id && upperAsk.CommCount == 1){
            P1X = upperAsk.coodiX;
            P1Y = upperAsk.coodiY;
            dist[0] = Receptor_Ranging((id - 1) % (n /2));
            if (dist[0] != 0xFFFF){
                if (id > 1 && id <= n/2 - 1){
                    channel = ChB;
                }
                if (id == 1){
                    channel = ChA;
                }
                if (id == n/2){
                    msg.Cmtype = secMeasure;
                    msg.Sender = n/2;
                    msg.RecieverL = P2;
                    msg.RecieverU = P2;
                    msg.CommCount = 0;
                    SendMessage(msg,5);
                    channel = ChB;//tell P2 that P1 and P2's measuring is done, start the trdstep, then id 2:n/2 reverse at ChB
                }
                //Calculate//
                double d, p, S, h, a, x_, y_;
                d = sqrt((P1X - P2X)*(P1X - P2X) + (P1Y - P2Y)*(P1Y - P2Y));
                p = (dist[0] + dist[1] + d) / 2;
                S = sqrt(p*(p-dist[0])*(p-dist[1])*(p-d));
                h = 2.0 * S / d;
                a = sqrt(dist[0]*dist[0] - h * h);
                x_ = a*1.0 / d * P2X + (1 - a*1.0 / d) * P1X;
                y_ = a*1.0 / d * P2Y + (1 - a*1.0 / d) * P1Y;
                x_1 = x_ + (P2Y - P1Y) * h / d;
                y_1 = y_ - (P2X - P1X) * h / d;
                x_2 = x_ - (P2Y - P1Y) * h / d;
                y_2 = y_ + (P2X - P1X) * h / d;
            }
        }
    }
    
    if (upperAsk.Cmtype == trdMeasure){
        //start the trdMeasure						     //
        if (upperAsk.Sender == P2 && upperAsk.RecieverL ==1 && upperAsk.RecieverU == n && upperAsk.CommCount == 0){
            if (id == 1){
                msg.Cmtype = trdMeasure;
                msg.Sender = 1;
                msg.RecieverL = n/2+2;
                msg.RecieverU = n;
                msg.coodiX = x_1;
                msg.coodiY = y_1;
                msg.Dist1 = x_2;
                msg.Dist2 = y_2;
                msg.CommCount = 1;
                SendMessage(msg,13);
                //start trdmeasuring with n/2+2:n
            }
            if (id == n/2+1){
                channel = ChB;
                msg.Cmtype = trdMeasure;
                msg.Sender = n/2+1;
                msg.RecieverL = 2;
                msg.RecieverU = n/2;
                msg.coodiX = x_1;
                msg.coodiY = y_1;
                msg.Dist1 = x_2;
                msg.Dist2 = y_2;
                msg.CommCount = 1;
                SendMessage(msg,13);
                //start trdmeasuring with 2:n/2
            }
        }
        
        //2:n/2 passive measure with n/2+1
        if (upperAsk.Sender == n/2 + 1 && upperAsk.RecieverL == 2 && upperAsk.RecieverU == n/2 && upperAsk.CommCount == 1){
            if (id >= 2 && id <= n/2){
                dist[3] = Receptor_Ranging(id-2);
                if (dist[3] != 0xFFFF){
                    double min11, min12, min21, min22, ID2X, ID2Y, min;
                    min11 = abs((msg.coodiY - y_1)*(msg.coodiY - y_1) + (msg.coodiX-x_1)*(msg.coodiX-x_1) - dist[3]*dist[3]);
                    min12 = abs((msg.coodiY - y_2)*(msg.coodiY - y_2) + (msg.coodiX-x_2)*(msg.coodiX-x_2) - dist[3]*dist[3]);
                    min21 = abs((msg.Dist2 - y_1)*(msg.Dist2 - y_1) + (msg.Dist1-x_1)*(msg.Dist1 - x_1) - dist[3]*dist[3]);
                    min22 = abs((msg.Dist2 - y_2)*(msg.Dist2 - y_2) + (msg.Dist1-x_2)*(msg.Dist1 - x_2) - dist[3]*dist[3]);
                    
                    x=x_1;
                    y=y_1;
                    ID2X = msg.coodiX;
                    ID2Y = msg.coodiY;
                    min = min11;
                    
                    if (min12 < min) {
                        min = min12;
                        x = x_1;
                        y = y_1;
                        ID2X = msg.coodiX;
                        ID2Y = msg.coodiY;
                    }	
                    if (min21 < min){
                        min = min21;
                        x = x_2;
                        y = y_2;
                        ID2X = msg.Dist1;
                        ID2Y = msg.Dist2;
                    }
                    if (min22 < min){
                        min = min22;
                        x = x_2;
                        y = y_2;
                        ID2X = msg.Dist1;
                        ID2Y = msg.Dist2;
                    }
                    if (id != n/2){
                        channel = ChA;//2:n/2 reverse channel at ChA
                    }
                    
                    if (id == n/2){//n/2 tell n/2+1 it's position
                        msg.Cmtype = trdMeasure;
                        msg.Sender = n/2;
                        msg.RecieverL = n/2+1;
                        msg.RecieverU = n/2+1;
                        msg.coodiX = ID2X;
                        msg.coodiY = ID2Y;
                        msg.Dist1 = 0;
                        msg.Dist2 = 0;
                        msg.CommCount = 0;
                        SendMessage(msg,13);
                        channel = ChA;
                    }
                }
            }
        }		
        //n/2+2:n passive measure with 1
        if (upperAsk.Sender == 1 && upperAsk.RecieverL == n/2 + 2 && upperAsk.RecieverU == n && upperAsk.CommCount == 1){
            if (id >= n/2 + 2 && id <= n){
                dist[3] =Receptor_Ranging(id-2-n/2);
                if (dist[3]!=0xFFFF){
                    double min11, min12, min21, min22, ID1X, ID1Y, min;
                    min11 = abs((msg.coodiY - y_1)*(msg.coodiY - y_1) + (msg.coodiX-x_1)*(msg.coodiX-x_1) - dist[3]*dist[3]);
                    min12 = abs((msg.coodiY - y_2)*(msg.coodiY - y_2) + (msg.coodiX-x_2)*(msg.coodiX-x_2) - dist[3]*dist[3]);
                    min21 = abs((msg.Dist2 - y_1)*(msg.Dist2 - y_1) + (msg.Dist1-x_1)*(msg.Dist1 - x_1) - dist[3]*dist[3]);
                    min22 = abs((msg.Dist2 - y_2)*(msg.Dist2 - y_2) + (msg.Dist1-x_2)*(msg.Dist1 - x_2) - dist[3]*dist[3]);
                    
                    x=x_1;
                    y=y_1;
                    ID1X = msg.coodiX;
                    ID1Y = msg.coodiY;
                    min = min11;
                    
                    if (min12 < min) {
                        min = min12;
                        x = x_1;
                        y = y_1;
                        ID1X = msg.coodiX;
                        ID1Y = msg.coodiY;
                    }	
                    if (min21 < min){
                        min = min21;
                        x = x_2;
                        y = y_2;
                        ID1X = msg.Dist1;
                        ID1Y = msg.Dist2;
                    }
                    if (min22 < min){
                        min = min22;
                        x = x_2;
                        y = y_2;
                        ID1X = msg.Dist1;
                        ID1Y = msg.Dist2;
                    }
                    
                    if (id == n){//n tell 1 it's position
                        msg.Cmtype = trdMeasure;
                        msg.Sender = n;
                        msg.RecieverL = 1;
                        msg.RecieverU = 1;
                        msg.coodiX = ID1X;
                        msg.coodiY = ID1Y;
                        msg.Dist1 = 0;
                        msg.Dist2 = 0;
                        msg.CommCount = 0;
                        SendMessage(msg,13);
                    }
                }
            }
        }
        
        
        //n/2 and n tell n/2+1 and 1 their XY position
        if (upperAsk.Sender == n && upperAsk.RecieverL == 1 && upperAsk.RecieverU == 1 && upperAsk.CommCount == 0){
            if (id == 1){
                x = msg.coodiX;
                y = msg.coodiY;
                
                // msg.Cmtype = trdMeasure;
                // msg.Sender = 1;
                // msg.RecieverL = P1;
                // msg.RecieverU = P1;
                // msg.CommCount = 0;
                // SendMsg(&data);
            }
        }
        
        if (upperAsk.Sender == n/2 && upperAsk.RecieverL == n/2+1 && upperAsk.RecieverU == n/2+1 && upperAsk.CommCount == 0){
            if (id == n/2+1){
                x = msg.coodiX;
                y = msg.coodiY;
                channel = ChA;//n/2+1 should reverse to ChA
                
                // msg.Cmtype = trdMeasure;
                // msg.Sender = n/2+1;
                // msg.RecieverL = P1;
                // msg.RecieverU = P1;
                // msg.CommCount = 0;
                // delay(1);
                // SendMsg(&data);
            }
        }
    }
}
 */
int main(int argc, char *argv[]){
    int id;
    int proc;
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &id);
    MPI_Comm_size(MPI_COMM_WORLD, &proc);
    
    
    p[0].id = 0;
    p[1].id = 1;
    p[2].id = 2;
    p[3].id = 3;
    p[0].pox = 0;
    p[0].poy = 0;
    p[1].pox = 100;
    p[1].poy = 0;
    p[2].pox = 50;
    p[2].poy = 400;
    p[3].pox = 500;
    p[3].poy = 480;
    if (id == 0){
        //printf("I'm %d", id);
        SGY_P1();
    }
    if (id == 1){
        //printf("I'm %d", id);
        SGY_P2();
    }
    if (id == 2){
        //printf("I'm %d", id);
        SGY_P3();
    }
    if (id == 3) {
        //printf("I'm %d", id);
        SGY_P4();
    }
    MPI_Finalize();
    return 0;
}
#endif

