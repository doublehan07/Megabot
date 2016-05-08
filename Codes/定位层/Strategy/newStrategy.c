#include "newStrategy.h"

void SGY_P1(){//P1's strategy
	
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE) //ÉÏÎ»»ú´ò¶ÏÁË¼àÌý
	{
		upperAsk = 0;
		return;
	}//listening...
	if recieve the indication from master{
		data[Cmtype] = Init;
		data[Cmtype]
	}

	if (data[Cmtype] == Init){
		if (data[Sender] == n){
			//start the four point locating...
			data[Cmtype] = fstMeasure;
			data[Sender] = P1;
			data[RecieverL] = P2;
			data[RecieverU] = P4;
			data[coodiX] = 0;
			data[coodiY] = 0;
			data[Dist1] = 0;
			data[Dist2] = 0;
			data[CommCount] = 1;
			Initiator_Ranging(&data);//Measuring P1 with P2 P3 P4 
		}
	}

	if (upperAsk[Cmtype] = secMeasure){
		if (upperAsk[Sender] == n/2 && upperAsk[RecieverL] == P1 && upperAsk[RecieverU] == P1 && upperAsk[Cmtype] == 0){
			data[Cmtype] = secMeasure;
			data[Sender] = P1;
			data[RecieverL] = 1;
			data[RecieverU] = n/2;
			data[coodiX] = x;
			data[coodiY] = y;
			Initiator_Ranging(&data);	
		}
	}

	// if (upperAsk[Cmtype] = endMeasure){
	// 	if (data[Sender] == 1 && data[RecieverL] == P1 && data[RecieverU] == P1){

	// 	}
	// }
}

void SGY_P2(){

	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); 
	if(upperAsk == 0xEE) 
	{
		upperAsk = 0;
		return;
	}//listening...

	if (upperAsk[Cmtype] = fstMeasure){
		if (upperAsk[Sender] == P1 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){//P1 measuring the distance with P2 P3 P4, CommCount==1 represent a measuring 
			dist = Receptor_Ranging(0);
			y = 0;
			x = dist;
			//get the position of P2
		}

		if (upperAsk[Sender] == P4 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P2 && upperAsk[CommCount] == 0){//P4 replies Measuring with P1, then tell P2 to start measuring P3, P4
			//then P2 start fstMeasure with P3 P4
			data[Cmtype] = fstMeasure;
			data[Sender] = P2;
			data[RecieverL] = P3;
			data[RecieverU] = P4;
			data[coodiX] = x;
			data[coodiY] = y;
			Initiator_Ranging(&data);
		}

		if (upperAsk[Sender] == P4 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P3 && upperAsk[CommCount] == 0){//P4 replies Measuring with P3, then tell P2 and P3 to start the secondstep
			//then P2 start secMeasure with 1:n/2
			data[Cmtype] = secMeasure;
			data[Sender] = P2;
			data[RecieverL] = 1;
			data[RecieverU] = n/2;
			data[coodiX] = x;
			data[coodiY] = y;
			Initiator_Ranging(&data);
		}
	}

	if (upperAsk[Cmtype] = secMeasure){
		if (upperAsk[Sender] == n/2 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P2 && upperAsk[CommCount] == 0){
			Done[0] = 1;//the measuring with 1:n/2 is done
		}  
		if (upperAsk[Sender] == n && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P2 && upperAsk[CommCount] == 0){
			Done[1] = 1;//the measuring with n/2+1:n is done
			if (Done[0] == 1 && Done[1] == 1){
				data[Cmtype] = trdMeasure;
				data[Sender] = P2;
				data[RecieverL] = 1;
				data[RecieverU] = n;
				//start the trdMeasure
				SendMsg(&data);
			}
		}  
	}
}

void SGY_P3(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //
	if(upperAsk == 0xEE){//
		upperAsk = 0;
		return;
	}//listening...
	if (upperAsk[Cmtype] == fstMeasure){
		if (upperAsk[Sender] == P1 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){
			dist[0] = Receptor_Ranging(1);
		}
		if (upperAsk[Sender] == P2 && upperAsk[RecieverL] == P3 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){
			dist[1] = Receptor_Ranging(0);
			double p, S;
			if (dist[1] != 0xFFFF && dist[2] != 0xFFFF)
			p = 1.0*(dist[1] + dist[2] + upperAsk[coodiY]) / 2.0;
			S = sqrt(p*(p-dist1)*(p-dist2)*(p-upperAsk[coodiX]));
			y = int(2 * S / upperAsk[coodiX]);
			x = int(sqrt(dist[1] * dist[1] - y * y));
			// Calculate P3's position
		}

		if (upperAsk[Sender] == P4 && upperAsk[RecieverL] == P3 && upperAsk[RecieverU] == P3 && upperAsk[CommCount] == 0){//P2 has measured P3 P4, then P4 should tell P3 to start measuring P3 P4
			data[Cmtype] = fstMeasure;
			data[Sender] = P3;
			data[RecieverL] = P4;
			data[RecieverU] = P4;
			data[coodiX] = x;
			data[coodiY] = y;
			data[Dist1] = 0;
			data[Dist2] = 0;
			Initiator_Ranging(&data);
		}
	}

	if (upperAskp[Cmtype] == secMeasure){
		if (upperAsk[Sender] == P4 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P3 && upperAsk[CommCount] == 0){//fstMeasuring is done then P3 should start the seconds step
			channel = ChB;
			data[Cmtype] = secMeasure;
			data[Sender] = P3;
			data[RecieverL] = n/2+1;
			data[RecieverU] = n;
			data[coodiX] = x;
			data[coodiY] = y;
			Initiator_Ranging(&data);
		}
	}
}

void SGY_P4(){
	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0); //µÈ´ý½ÓÊÕµ½ÏûÏ¢
	if(upperAsk == 0xEE){
		upperAsk = 0;
		return;
	}//listening...

	if (upperAsk[Cmtype] == fstMeasure){
		if (upperAsk[Sender] == P1 && upperAsk[RecieverL] == P2 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){
			dist[0] = Receptor_Ranging(2);
		}
		if (upperAsk[Sender] == P2 && upperAsk[RecieverL] == P3 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){
			P2X = upperAsk[coodiX];
			dist[1] = Receptor_Ranging(1);
		}
		if (upperAsk[Sender] == P3 && upperAsk[RecieverL] == P4 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 1){
			dist[2] = Receptor_Ranging(0);
			if (dist[2] != 0xFFFF && dist[1] != 0xFFFF && dist[0] != 0xFFFF){
				double p, S;
				p = 1.0*(dist[1] + dist[2] + P2X) / 2.0;
				S = sqrt(p*(p-dist1)*(p-dist2)*(p-P2X));
				y = int(2 * S / P2X);
				x = int(sqrt(dist[1] * dist[1] - y * y));
				if ((x-upperAsk[coodiX])*(x-upperAsk[coodiX])+(y-upperAsk[coodiY])*(x-upperAsk[coodiY]) - dist[3]*dist[3] >
					(x-upperAsk[coodiX])*(x-upperAsk[coodiX])+(y+upperAsk[coodiY])*(x+upperAsk[coodiY]) - dist[3]*dist[3]){
					y = -y;
				// Calculate P4's position
				}
				//fstMeasure completed, send message to let P2 and P3 to start secMeasure
				data[Cmtype] = secMeasure;
				data[Sender] = P4;
				data[RecieverL] = P2;
				data[RecieverU] = P3;
				data[coodiX] = 0;
				data[coodiY] = 0;
				data[Dist1] = 0;
				data[Dist2] = 0;
				data[CommCount] = 0;
				SendMsg(&data);
				channel = ChB;
			}
		}
	}

	if (upperAsk[Cmtype] == secMeasure){
		if (upperAsk[Sender] == n && upperAsk[RecieverL] == P4 && upperAsk[RecieverU] == P4 && upperAsk[CommCount] == 0){
			//secMeasure of P3 is done, it's turn to P4 to measuring n/2+1:n;
			data[Cmtype] = secMeasure;
			data[Sender] = P4;
			data[RecieverL] = n/2+1;
			data[RecieverU] = n;
			data[coodiX] = x;
			data[coodiY] = y;
		}
	}
}

void SGY_Receptor(){
	//variances list
	static int x;
	static int y;
	static int x1,x2,y1,y2;
	static int P1X, P1Y, P2X, P2Y, P3X, P3Y, P4X, P4Y;
	u8 upperAsk[];

	do{
		upperAsk = Receptor_Listening(0);
	}
	while(upperAsk == 0);
	if(upperAsk == 0xEE) 
		upperAsk = 0;
		return;
	}
	
	//Listening...
	if (upperAsk[Cmtype] == Init){
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
		delay(id - 1);
		//tell Leader Init is done
		SendMsg(&data)
		if (id > n/2){
			channel = ChB;
		}
		else{
			channel = ChA;
		}
	}

	if (upperAsk[Cmtype] == secMeasure){															//==n/2+1 						//==n
		if (upperAsk[Sender] == P3 && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >= id && upperAsk[CommCount] == 1){//P3 done, let P4 start measuring
			P3X = upperAsk[coodiX];
			P3Y = upperAsk[coodiY];
			dist[0] = Receptor_Ranging((id-1) % (n/2));
			if (id == n && dist[0] != 0xFFFF){
				data[Cmtype] = secMeasure;
				data[Sender] = n;
				data[RecieverL] = P4;
				data[RecieverU] = P4;
				data[CommCount] = 0;
				SendMsg(&data);
			} 
		}												//==n/2+1 						//==n
		if (upperAsk[Sender] == P4 && upperAsk[RecieverL] <= id  && upperAsk[RecieverU] >= id && upperAsk[CommCount] == 1){//P4 measure with n/2+1 : n
			P4X = upperAsk[coodiX];
			P4Y = upperAsk[coodiY];
			dist[1] = Receptor_Ranging((id-1) % (n/2));
			if (dist[1] != 0xFFFF){
				if (id >= n/2 + 1){// if this measuring is done, n/2+2:n should reverse channel to ChA
					channel = ChA;
				}
				if (id == n){
					data[Cmtype] = secMeasure;
					data[Sender] = n;
					data[RecieverL] = P2;
					data[RecieverU] = P2;
					data[CommCount] = 0;
					SendMsg(&data);
				}
				double d, p, S, h, a, x_, y_;
				d = sqrt((P3X - P4X)*(P3X - P4X) + (P3Y - P4Y)*(P3Y - P4Y));
				p = (dist1 + dist2 + d) / 2;
				S = sqrt(p*(p-dist1)*(p-dist2)*(p-d));
				h = 2 * S / d;
				a = sqrt(dist1*dist1 - h * h);
				x_ = dist1 / d * P4X + (1 - dist1 / d) * P3X;
				y_ = dist2 / d * P4Y + (1 - dist1 / d) * P3Y;
				x1 = x_ + (P4X - P3X) * h / d;
				y1 = y_ - (P4Y - P3Y) * h / d;
				x2 = x_ - (P4X - P3X) * h / d;
				y2 = y_ + (P4Y - P3Y) * h / d; 
			}
		}

		if (upperAsk[Sender] == P2 && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >= id && upperAsk[CommCount] == 1){
			P2X = upperAsk[coodiX];
			P2Y = upperAsk[coodiY];
			dist[1] = Receptor_Ranging((id - 1) % (n / 2));
            if (id == n/2 && dist[1] != 0xFFFF){
            	data[Cmtype] = secMeasure;
				data[Sender] = n/2;
				data[RecieverL] = P1;
				data[RecieverU] = P1;
				data[CommCount] = 0;
				SendMsg(&data);
            }
		}

		if (upperAsk[Sender] == P1 && upperAsk[RecieverL] <= id && upperAsk[RecieverU] >= id && upperAsk[CommCount] == 1){
			P1X = upperAsk[coodiX];
			P1Y = upperAsk[coodiY];
			data[0] = Receptor_Ranging((id - 1) % (n /2));
			if (dist[0] != 0xFFFF){
				if (id > 1 && id <= n/2 - 1){
					channel = ChB;
				}
				if (id == 1){
					channel = ChA;
				}
				if (id == n/2){
					data[Cmtype] = secMeasure;
					data[Sender] = n/2;
					data[RecieverL] = P2;
					data[RecieverU] = P2;
					data[CommCount] = 0;
					SendMsg(&data);
					channel = ChB;//tell P2 that P1 and P2's measuring is done, start the trdstep
				}
				//Calculate//	
				double d, p, S, h, a, x_, y_;
				d = sqrt((P1X - P2X)*(P1X - P2X) + (P1Y - P2Y)*(P1Y - P2Y));
				p = (dist1 + dist2 + d) / 2;
				S = sqrt(p*(p-dist1)*(p-dist2)*(p-d));
				h = 2 * S / d;
				a = sqrt(dist1*dist1 - h * h);
				x_ = dist1 / d * P2X + (1 - dist1 / d) * P1X;
				y_ = dist2 / d * P2Y + (1 - dist1 / d) * P1Y;
				x1 = x_ + (P2X - P1X) * h / d;
				y1 = y_ - (P2Y - P1Y) * h / d;
				x2 = x_ - (P2X - P1X) * h / d;
				y2 = y_ + (P2Y - P1Y) * h / d;
			}
		}
	}

	if (upperAsk[Cmtype] == trdMeasure){
		//start the trdMeasure						     //		
		if (upperAsk[Sender] == P2 && upperAsk[RecieverL] ==1 && upperAsk[RecieverU] == n && upperAsk[CommCount] == 0){
			if (id == 1){
				data[Cmtype] = trdMeasure;
				data[Sender] = 1;
				data[RecieverL] = n/2+2;
				data[RecieverU] = n;
				data[coodiX] = x1;
				data[coodiY] = y1;
				data[Dist1] = x2;
				data[Dist2] = y2;
				data[CommCount] = 1;
				SendMsg(&data);
				//start trdmeasuring with n/2+2:n
			}
			if (id == n/2+1){
				channel = ChB;
				data[Cmtype] = trdMeasure;
				data[Sender] = n/2+1;
				data[RecieverL] = 2;
				data[RecieverU] = n/2;
				data[coodiX] = x1;
				data[coodiY] = y1;
				data[Dist1] = x2;
				data[Dist2] = y2;
				data[CommCount] = 1;
				SendMsg(&data);
				//start trdmeasuring with 2:n/2
 			}
		}

		//2:n/2 passive measure with n/2+1
		if (upperAsk[Sender] == n/2 + 1 && upperAsk[RecieverL] == 2 && upperAsk[RecieverU] == n/2 && upperAsk[CommCount] == 1){
			if (id >= 2 && id <= n/2){
				dist=Receptor_Ranging(id-2);
				if (dist != 0xFFFF){
					double min11, min12, min21, min22, ID2X, ID2Y;
					min11 = abs((data[coodiY] - y1)*(data[coodiY] - y1) + (data[coodiX]-x1)*(data[coodiX]-x1) - dist*dist);
					min12 = abs((data[coodiY] - y2)*(data[coodiY] - y2) + (data[coodiX]-x2)*(data[coodiX]-x2) - dist*dist);
					min21 = abs((data[Dist2] - y1)*(data[Dist2] - y1) + (data[Dist1]-x1)*(data[Dist1] - x1) - dist*dist);
					min22 = abs((data[Dist2] - y2)*(data[Dist2] - y2) + (data[Dist1]-x2)*(data[Dist1] - x2) - dist*dist);

					x=x1;
					y=y1;
					ID2X = data[coodiX];
					ID2Y = data[coodiY];
					min = min11;

					if (min12 < min) {
						min = min12;
						x = x1;
						y = y1;
						ID2X = data[coodiX];
						ID2Y = data[coodiY];
					}	
					if (min21 < min){
						min = min21;
						x = x2;
						y = y2;
						ID2X = data[Dist1];
						ID2Y = data[Dist2];
					}
					if (min22 < min){
						min = min22;
						x = x2;
						y = y2;
						ID2X = data[Dist1];
						ID2Y = data[Dist2];
					}
					if (id != n/2){
						channel = ChA;//2:n/2 reverse channel at ChA
					}
				
					if (id == n/2){
						data[Cmtype] = trdMeasure;
						data[Sender] = n/2;
						data[RecieverL] = n/2+1;
						data[RecieverU] = n/2+1;
						data[coodiX] = ID2X;
						data[coodiY] = ID2Y;
						data[Dist1] = 0;
						data[Dist2] = 0;
						data[CommCount] = 0;
						SendMsg(&data);
						channel = ChA;
					}
				}
			}
		}		
		//n/2+2:n passive measure with 1
		if (upperAsk[Sender] == 1 && upperAsk[RecieverL] == n/2 + 2 && upperAsk[RecieverU] == n && upperAsk[CommCount] == 1){
			if (id >= n/2 + 2 && id <= n){
				dist=Receptor_Ranging(id-2-n/2);
				if (dist!=0xFFFF){
					double min11, min12, min21, min22, ID1X, ID1Y;
					min11 = abs((data[coodiY] - y1)*(data[coodiY] - y1) + (data[coodiX]-x1)*(data[coodiX]-x1) - dist*dist);
					min12 = abs((data[coodiY] - y2)*(data[coodiY] - y2) + (data[coodiX]-x2)*(data[coodiX]-x2) - dist*dist);
					min21 = abs((data[Dist2] - y1)*(data[Dist2] - y1) + (data[Dist1]-x1)*(data[Dist1] - x1) - dist*dist);
					min22 = abs((data[Dist2] - y2)*(data[Dist2] - y2) + (data[Dist1]-x2)*(data[Dist1] - x2) - dist*dist);

					x=x1;
					y=y1;
					ID1X = data[coodiX];
					ID1Y = data[coodiY];
					min = min11;

					if (min12 < min) {
						min = min12;
						x = x1;
						y = y1;
						ID1X = data[coodiX];
						ID1Y = data[coodiY];
					}	
					if (min21 < min){
						min = min21;
						x = x2;
						y = y2;
						ID1X = data[Dist1];
						ID1Y = data[Dist2];
					}
					if (min22 < min){
						min = min22;
						x = x2;
						y = y2;
						ID1X = data[Dist1];
						ID1Y = data[Dist2];
					}
				
					if (id == n){
						data[Cmtype] = trdMeasure;
						data[Sender] = n;
						data[RecieverL] = 1;
						data[RecieverU] = 1;
						data[coodiX] = ID1X;
						data[coodiY] = ID1Y;
						data[Dist1] = 0;
						data[Dist2] = 0;
						data[CommCount] = 0;
						SendMsg(&data);
					}
				}
			}
		}


		//n/2 and n tell n/2+1 and 1 their XY position
		if (upperAsk[Sender] == n && upperAsk[RecieverL] == 1 && upperAsk[RecieverU] == 1 && upperAsk[CommCount] == 0){
			if (id == 1){
				x = data[coodiX];
				y = data[coodiY];
				
				// data[Cmtype] = trdMeasure;
				// data[Sender] = 1;
				// data[RecieverL] = P1;
				// data[RecieverU] = P1;
				// data[CommCount] = 0;
				// SendMsg(&data);
			}
		}

		if (upperAsk[Sender] == n/2 && upperAsk[RecieverL] == n/2+1 && upperAsk[RecieverU] == n/2+1 && upperAsk[CommCount] == 0){
			if (id == n/2+1){
				x = data[coodiX];
				y = data[coodiY];
				channel = ChA;//n/2+1 should reverse to ChA

				// data[Cmtype] = trdMeasure;
				// data[Sender] = n/2+1;
				// data[RecieverL] = P1;
				// data[RecieverU] = P1;
				// data[CommCount] = 0;
				// delay(1);
				// SendMsg(&data);
			}
		}
	}
}


