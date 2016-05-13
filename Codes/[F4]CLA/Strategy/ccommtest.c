//
//  Commtest.c
//  testMPI
//
//  Created by 温拓朴 on 16/5/9.
//  Copyright © 2016年 温拓朴. All rights reserved.
//
/*
#ifndef COMM
#define COMM

#include "Commtest.h"

extern Node p[4];
extern int dist[4][4];
extern int get_all_distance;

void SendMsg(u8*, int);
void InitiatorMeasuring(u8*, int);
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
            SendMsg(data, len);
        }
        else{
            InitiatorMeasuring(data, len);
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

void SGY_P1(){ //P1's strategy   
    Message upperAsk;
    Message msg;
    u8 len;
    if (flag == 1){
        msg.Cmtype = Init;
        msg.Sender = P1;
        msg.RecieverL = P2;
        msg.RecieverU = P4;
        msg.CommCount = 0;
        SendMessage(msg, 5);
			  p[0].x = 0;
				p[0].y = 0;
				flag = 0;
    }
    
		Receptor_Listening(p[0].rx_buffer, &len);
    upperAsk = SaveMessage(p[0].rx_buffer, len);
		
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
    
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        p[0].dist[0] = Receptor_Ranging(&p[0],&p[1]);
        //get the distance of P1 and P2
    }
    
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        p[0].dist[1] = Receptor_Ranging(&p[0],&p[2]);
        //get the distance of P1 and P3
    }
    
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
        p[0].dist[2] = Receptor_Ranging(&p[0],&p[3]);
			  //get the distance of P1 and P4
        msg.Cmtype = fstMeasure;
        msg.Sender = P1;
        msg.RecieverL = P4;
        msg.RecieverU = P4;
        msg.coodiX = p[0].dist[0];
        msg.coodiY = p[0].dist[1];
        msg.Dist1 = p[0].dist[2];
        msg.Dist2 = 0;
        msg.CommCount = 0;
        SendMessage(msg, 13);
        //tell P4 the measuring result
    }
}

void SGY_P2(){
		u8 len;
    Message msg,upperAsk;  
    Receptor_Listening(p[1].rx_buffer, &len);
    upperAsk = SaveMessage(p[1].rx_buffer, len);
	
    if (upperAsk.Cmtype == Init){
			    p[1].x = 0;
					p[1].y = 0;

        //printf("P2 has Initialed\n");
    }

				if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
            p[1].dist[0] = Receptor_Ranging(&p[1],&p[0]);         
            //get the distance of P1 and P2
        }
    
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){//P4 replies Measuring with P1, then tell P2 to start measuring P3, P4
            //then P2 start fstMeasure with P1 P3 P4
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
    
				if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P3 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
						p[1].dist[1] = Receptor_Ranging(&p[1],&p[2]);
        //get the distance of P3 and P2
				}
    
  
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
				//tell P4 the result
				}
    
				if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){
						p[1].x = upperAsk.coodiX;
						p[1].y = upperAsk.coodiY;
        //get the calculating result of P2;
				}
}

void SGY_P3(){
    Message msg, upperAsk;
    u8 len;
    ReceptorGrasping(p[2].rx_buffer, &len);
    upperAsk = SaveMessage(p[2].rx_buffer, len);
	
    if (upperAsk.Cmtype == Init){
			    p[2].channel = ChA;
					p[2].x = 0;
					p[2].y = 0;
        //printf("P2 has Initialed\n");
    }
    
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[2].dist[0] = Receptor_Ranging(&p[2], &p[0]);
        }//measuring P1 with P2 P3 P4
    
    //mesuring P2 with P3 P4
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[2].dist[1] = Receptor_Ranging(&p[2], &p[1]);
        }
    
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
					SendMessage(msg,13);
			}//tell P4 the measuring result 
    
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){
        p[2].x = upperAsk.Dist1;
        p[2].y = upperAsk.Dist2;
        //get the result of P4;
    }
}

void SGY_P4(){
    Message msg, upperAsk;
    int i,j;
    u8 len;
    Receptor_Listening(p[3].rx_buffer,&len);
    upperAsk = SaveMessage(p[3].rx_buffer, len);

    if (upperAsk.Cmtype == Init){
			  p[3].x = 0;
				p[3].y = 0;
				get_all_distance = 0;
	for (i = 0; i < 4; i++)
		for (j = 0; j < 4; j++){
			dist[i][j] = 0;
	}

        msg.Cmtype = Init;
        msg.Sender = P4;
        msg.RecieverL = P1;
        msg.RecieverU = P1;
        msg.CommCount = 0;
        SendMessage(msg, 5);
    }
    
   
        if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
            p[3].dist[0] = Receptor_Ranging(&p[3],&p[0]);
            msg.Cmtype = fstMeasure;
            msg.Sender = P4;
            msg.RecieverL = P2;
            msg.RecieverU = P2;
            msg.CommCount = 0;
            SendMessage(msg,5);//tell P2 to Start
        }
    
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
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P1 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[0][1] = upperAsk.coodiX;//P12
        dist[0][2] = upperAsk.coodiY;//P13
        dist[0][3] = upperAsk.Dist1;//P14
    }
    
    //get result from P2
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P2 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[1][0] = upperAsk.coodiX;//P21
        dist[1][2] = upperAsk.coodiY;//P23
        dist[1][3] = upperAsk.Dist1;//P24
    }
    
    //get result from P3
    if (upperAsk.Cmtype == fstMeasure && upperAsk.Sender == P3 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
        dist[2][0] = upperAsk.coodiX;//P31
        dist[2][1] = upperAsk.coodiY;//P32
        dist[2][3] = upperAsk.Dist1;//P34
				get_all_distence == 1;
    }
		
    //calculate!
		if (get_all_distance == 1){
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
		}
}

#endif
*/
