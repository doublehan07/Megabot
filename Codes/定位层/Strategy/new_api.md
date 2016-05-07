##API

###data

**Cmtype**:              fstMeasure:0xA sndMeasure:0xB trdMeasure:0xC  
**SenderID**:			  id who send the message   
**RecieveIDLowerBound**: LowerBound<=id who should recieve the message   
**RecieveIDUpperBound**: UpperBound>=id who should recieve the message   
**CoodinateX**:			  		   
**CoodinateY**:   	      
**Dist1**:               if P1,P3 then result is saved in Dist1  
**Dist2**:               if P2,P4 then result is saved in Dist2    
**CommCount**:           how many times have contacted    
**FLAG**:                SUCCESS:0x01 FAIL:0x00

**Message length**:       

void SGY_Sendmessage(&data)
//Boardcast data

void SGY_Measuring_Distance(&data)
//Measuring 

u32 active_measuring()
u32 passive_measuring(int delay_ms)   
//return the message in poll   
//这两个名字按你的写，就是被动测距函数需要传进一个延时的毫秒数

