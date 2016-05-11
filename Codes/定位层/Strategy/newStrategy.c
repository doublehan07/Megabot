
#define P1 (n+1)
#define P2 (n+2)
#define P3 (n+3)
#define P4 (n+4)
#define Init 0x01
#define Finale 0x0F
#define fstMeasure 0x0A
#define secMeasure 0x0B
#define trdMeasure 0x0C
#define ChA 1
#define ChB 1

typedef short u8;
typedef unsigned int u16;
int n;
int x, y;
int dist[4];
int channel;
int id;
int x1,x2,y1,y2;
int P1X, P1Y, P2X, P2Y, P3X, P3Y, P4X, P4Y;

u8 rx_buffer[30];
int Done[2];

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

void SendMsg(u8*, int);
void Initiator_Ranging(u8*, int);
int Receptor_Listening(int);
int Receptor_Ranging(int);

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
		if (msg.CommCount == 0)
			SendMsg(data, len);
		else 
			Initiator_Ranging(data, len);
		return data;
	}
	else{
        u8 data[5];
		data[0] = msg.Cmtype;
		data[1] = msg.Sender;
		data[2] = msg.RecieverL;
		data[3] = msg.RecieverU;
		data[4] = msg.CommCount;
		SendMsg(data, len);
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

void SGY_P1(){//P1's strategy
	Message upperAsk;
	Message msg;
    int len;
    int flag;
	while ((len = Receptor_Listening(0)) != 0);
	upperAsk = SaveMessage(rx_buffer, len);
	if (flag == 1){
		msg.Cmtype = Init;
		msg.Sender = P1;
		msg.RecieverL = 1;
		msg.RecieverU = P4;
		msg.CommCount = 0;
		SendMessage(msg, 5);
	}

	if (upperAsk.Cmtype == Init){
		if (upperAsk.Sender == n){
			//start the four point locating...
			msg.Cmtype = fstMeasure;
			msg.Sender = P1;
			msg.RecieverL = P2;
			msg.RecieverU = P4;
			msg.coodiX = 0;
			msg.coodiY = 0;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);//Measuring P1 with P2 P3 P4 
		}
	}

	if (upperAsk.Cmtype == secMeasure){
		//P1 measuring with 1:n/2
		if (upperAsk.Sender == n/2 && upperAsk.RecieverL == P1 && upperAsk.RecieverU == P1 && upperAsk.Cmtype == 0){
			msg.Cmtype = secMeasure;
			msg.Sender = P1;
			msg.RecieverL = 1;
			msg.RecieverU = n/2;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);	
		}
	}

	// if (upperAsk.Cmtype] = endMeasure){
	// 	if (msg.Sender] == 1 && msg.RecieverL] == P1 && msg.RecieverU] == P1){

	// 	}
	// }
}

void SGY_P2(){

	Message msg, upperAsk;
	int len;
    
	while ((len = Receptor_Listening(0)) != 0);
	upperAsk = SaveMessage(rx_buffer, len);
    
    if (upperAsk.Cmtype == Init){
        Done[0] = 0;
        Done[1] = 0;
    }

	if (upperAsk.Cmtype == fstMeasure){
		if (upperAsk.Sender == P1 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring
            dist[0] = Receptor_Listening(0);
			y = 0;
            x = dist[0];
			//get the position of P2
		}

		if (upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){//P4 replies Measuring with P1, then tell P2 to start measuring P3, P4
			//then P2 start fstMeasure with P3 P4
			msg.Cmtype = fstMeasure;
			msg.Sender = P2;
			msg.RecieverL = P3;
			msg.RecieverU = P4;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);
		}

		if (upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//P4 replies Measuring with P3, then tell P2 and P3 to start the secondstep
			//then P2 start secMeasure with 1:n/2
			msg.Cmtype = secMeasure;
			msg.Sender = P2;
			msg.RecieverL = 1;
			msg.RecieverU = n/2;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);
		}
	}

	if (upperAsk.Cmtype == secMeasure){
		if (upperAsk.Sender == n/2 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){
			Done[0] = 1;//the measuring with 1:n/2 is done
		}  
		if (upperAsk.Sender == n && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P2 && upperAsk.CommCount == 0){
			Done[1] = 1;//the measuring with n/2+1:n is done
			if (Done[0] == 1 && Done[1] == 1){
				msg.Cmtype = trdMeasure;
				msg.Sender = P2;
				msg.RecieverL = 1;
				msg.RecieverU = n;
				msg.CommCount = 0;
				//start the trdMeasure
				SendMessage(msg,5);
			}
		}  
	}
}

void SGY_P3(){
	Message msg, upperAsk;
	int len;
    
	while ((len = Receptor_Listening(0)) != 0);
	upperAsk = SaveMessage(rx_buffer, len);

	if (upperAsk.Cmtype == fstMeasure){
		if (upperAsk.Sender == P1 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
			dist[0] = Receptor_Ranging(1);
		}
		if (upperAsk.Sender == P2 && upperAsk.RecieverL == P3 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
			dist[1] = Receptor_Ranging(0);
			double p, S;
            if (dist[0] != 0xFFFF && dist[1] != 0xFFFF){
                p = 1.0*(dist[1] + dist[0] + upperAsk.coodiY) / 2.0;
                S = sqrt(p*(p-dist[1])*(p-dist[0])*(p-upperAsk.coodiX));
                y = (int)(2 * S/upperAsk.coodiX);
                x = (int)(sqrt(dist[0] * dist[0] - y * y));
            }
			// Calculate P3's position
		}

		if (upperAsk.Sender == P4 && upperAsk.RecieverL == P3 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//P2 has measured P3 P4, then P4 should tell P3 to start measuring P3 P4
			msg.Cmtype = fstMeasure;
			msg.Sender = P3;
			msg.RecieverL = P4;
			msg.RecieverU = P4;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);
		}
	}

	if (upperAsk.Cmtype == secMeasure){
		if (upperAsk.Sender == P4 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P3 && upperAsk.CommCount == 0){//fstMeasuring is done then P3 should start the seconds step
			channel = ChB;
			msg.Cmtype = secMeasure;
			msg.Sender = P3;
			msg.RecieverL = n/2+1;
			msg.RecieverU = n;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);
		}
	}
}

void SGY_P4(){
    Message msg, upperAsk;
	int len;
	while ((len = Receptor_Listening(0)) != 0);
	upperAsk = SaveMessage(rx_buffer, len);

	if (upperAsk.Cmtype == fstMeasure){
		if (upperAsk.Sender == P1 && upperAsk.RecieverL == P2 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
			dist[0] = Receptor_Ranging(2);
		}
		if (upperAsk.Sender == P2 && upperAsk.RecieverL == P3 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
			P2X = upperAsk.coodiX;
			dist[1] = Receptor_Ranging(1);
		}
		if (upperAsk.Sender == P3 && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 1){
			dist[2] = Receptor_Ranging(0);
			if (dist[2] != 0xFFFF && dist[1] != 0xFFFF && dist[0] != 0xFFFF){
				double p, S;
				p = 1.0*(dist[1] + dist[2] + P2X) / 2.0;
				S = sqrt(p*(p-dist[1])*(p-dist[2])*(p-P2X));
				y = (int)(2 * S / P2X);
				x = (int)(sqrt(dist[1] * dist[1] - y * y));
				if ((x-upperAsk.coodiX)*(x-upperAsk.coodiX)+(y-upperAsk.coodiY)*(x-upperAsk.coodiY) - dist[3]*dist[3] >
					(x-upperAsk.coodiX)*(x-upperAsk.coodiX)+(y+upperAsk.coodiY)*(x+upperAsk.coodiY) - dist[3]*dist[3]){
					y = -y;
				// Calculate P4's position
				}
				//fstMeasure completed, send message to let P2 and P3 to start secMeasure
				msg.Cmtype = secMeasure;
				msg.Sender = P4;
				msg.RecieverL = P2;
				msg.RecieverU = P3;
				msg.coodiX = 0;
				msg.coodiY = 0;
				msg.Dist1 = 0;
				msg.Dist2 = 0;
				msg.CommCount = 0;
				SendMessage(msg,13);
				channel = ChB;
			}
		}
	}

	if (upperAsk.Cmtype == secMeasure){
		if (upperAsk.Sender == n && upperAsk.RecieverL == P4 && upperAsk.RecieverU == P4 && upperAsk.CommCount == 0){
			//secMeasure of P3 is done, it's turn to P4 to measuring n/2+1:n;
			msg.Cmtype = secMeasure;
			msg.Sender = P4;
			msg.RecieverL = n/2+1;
			msg.RecieverU = n;
			msg.coodiX = x;
			msg.coodiY = y;
			msg.Dist1 = 0;
			msg.Dist2 = 0;
			msg.CommCount = 1;
			SendMessage(msg,13);
		}
	}
}

void SGY_Receptor(){
	//variances list
	Message msg,upperAsk;
	int len;
	while ((len = Receptor_Listening(0)) != 0);
	upperAsk = SaveMessage(rx_buffer, len);
	//Listening...


	if (upperAsk.Cmtype == Init){
		P1X = 0;
		P2X = 0;
		P3X = 0;
		P4X = 0;
		x = 0;
		y = 0;
		x1 = 0;
		x2 = 0;
		y1 = 0;
		y2 = 0;
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
                x1 = x_ + (P4Y - P3Y) * h / d;
                y1 = y_ - (P4X - P3X) * h / d;
                x2 = x_ - (P4Y - P3Y) * h / d;
                y2 = y_ + (P4X - P3X) * h / d;
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
                x1 = x_ + (P2Y - P1Y) * h / d;
                y1 = y_ - (P2X - P1X) * h / d;
                x2 = x_ - (P2Y - P1Y) * h / d;
                y2 = y_ + (P2X - P1X) * h / d;
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
				msg.coodiX = x1;
				msg.coodiY = y1;
				msg.Dist1 = x2;
				msg.Dist2 = y2;
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
				msg.coodiX = x1;
				msg.coodiY = y1;
				msg.Dist1 = x2;
				msg.Dist2 = y2;
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
					min11 = abs((msg.coodiY - y1)*(msg.coodiY - y1) + (msg.coodiX-x1)*(msg.coodiX-x1) - dist[3]*dist[3]);
					min12 = abs((msg.coodiY - y2)*(msg.coodiY - y2) + (msg.coodiX-x2)*(msg.coodiX-x2) - dist[3]*dist[3]);
					min21 = abs((msg.Dist2 - y1)*(msg.Dist2 - y1) + (msg.Dist1-x1)*(msg.Dist1 - x1) - dist[3]*dist[3]);
					min22 = abs((msg.Dist2 - y2)*(msg.Dist2 - y2) + (msg.Dist1-x2)*(msg.Dist1 - x2) - dist[3]*dist[3]);

					x=x1;
					y=y1;
					ID2X = msg.coodiX;
					ID2Y = msg.coodiY;
					min = min11;

					if (min12 < min) {
						min = min12;
						x = x1;
						y = y1;
						ID2X = msg.coodiX;
						ID2Y = msg.coodiY;
					}	
					if (min21 < min){
						min = min21;
						x = x2;
						y = y2;
						ID2X = msg.Dist1;
						ID2Y = msg.Dist2;
					}
					if (min22 < min){
						min = min22;
						x = x2;
						y = y2;
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
					min11 = abs((msg.coodiY - y1)*(msg.coodiY - y1) + (msg.coodiX-x1)*(msg.coodiX-x1) - dist[3]*dist[3]);
					min12 = abs((msg.coodiY - y2)*(msg.coodiY - y2) + (msg.coodiX-x2)*(msg.coodiX-x2) - dist[3]*dist[3]);
					min21 = abs((msg.Dist2 - y1)*(msg.Dist2 - y1) + (msg.Dist1-x1)*(msg.Dist1 - x1) - dist[3]*dist[3]);
					min22 = abs((msg.Dist2 - y2)*(msg.Dist2 - y2) + (msg.Dist1-x2)*(msg.Dist1 - x2) - dist[3]*dist[3]);

					x=x1;
					y=y1;
					ID1X = msg.coodiX;
					ID1Y = msg.coodiY;
					min = min11;

					if (min12 < min) {
						min = min12;
						x = x1;
						y = y1;
						ID1X = msg.coodiX;
						ID1Y = msg.coodiY;
					}	
					if (min21 < min){
						min = min21;
						x = x2;
						y = y2;
						ID1X = msg.Dist1;
						ID1Y = msg.Dist2;
					}
					if (min22 < min){
						min = min22;
						x = x2;
						y = y2;
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


