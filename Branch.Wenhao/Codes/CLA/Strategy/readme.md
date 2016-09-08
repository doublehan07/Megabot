#通信流程
##四个子节点的定位
	- P1(Leader)广播，激活网络测距
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC
	0x0A | 0x01 | 0x02 | 0x04 | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 | 0x0D | 0x0A

	- P2-P4回复
	Cmdtype | SenderID | TargetID | CRC | CRC
	0x1A | SenderID } 0x01 | 0x0D | 0x0A

	- P1广播
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 3 times
	0x2A | 0x01 | TargetID | ts[4:1] | 0x0D | 0x0A

	//注意校验，这里不要最后一次回复

	- P2广播
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC
	0x0A | 0x02 | 0x03 | 0x04 | X | 0x00 | 0x00 | 0x00 | 0x00 | 0x0D | 0x0A

	- P3-P4回复
	CmdType | SenderID | TargetID | CRC | CRC
	0x1A | SenderID | 0x02 | 0x0D | 0x0A

	- P2广播
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 2 times
	0x2A | 0x02 | TargetID | ts[4:1] | 0x0D | 0x0A

	//注意校验，这里不要最后一次回复

	- P3广播
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1 | Dist2 | FLAG(RESV) | CRC | CRC
	0x0A | 0x03 | 0x04 | 0x04 | X1 | Y1 | X2 | Y2 | 0x00 | 0x0D | 0x0A

	- P4回复
	CmdType | SenderID | TargetID | CRC | CRC
	0x1A | 0x04 | 0x03 | 0x0D | 0x0A

	- P3广播
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 2 times
	0x2A | 0x03 | 0x04 | ts[4:1] | 0x0D | 0x0A

	//注意校验，这里不要最后一次回复

	**Interface**
	u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)

##四节点的后续操作
	- P4计算出自己和P3的坐标，同时知道P1和P2的坐标
	- P4向全网络N个节点广播P1-P4坐标信息
	- P4告诉P2,P3开始测距
	- P3告诉节点们翻转到B频段
	**Interface**
	u8 SendMsg(u8 *pointer, u8 arrayLenth), return IF_SUCCESS_SENDING(0x00 - T, 0xFF - F)

##第二次广播测距 - 0x0B
	- P2发起第二次广播测距
	Cmtype | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
		   | CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC

	0x0B | SenderID | LBD | UBD | X | Y | D1 | D2 | FLAG | 0x0D | 0x0A
		   
	- Px回复
	Cmdtype | SenderID | TargetID | CRC | CRC
	0xB1 | SenderID | TargetID | 0x0D | 0x0A

	- P2广播
	Cmdtype | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - n times
	0xB2 | SenderID | TargetID | ts[4:1] | 0x0D | 0x0A

	- 最后一个节点回复
	Cmdtype | SenderID | TargetID | LocalizationStatus | dist[2:1] | CRC | CRC
	0xB3 | SenderID | TargetID | TorF | distwithP2[2:1] | 0x0D | 0x0A

	**Interface**
	- u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	- u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)
	
##下次广播测距之前的同步处理
	- P3收到最后一个点回复之后，通知P4开始测距，同时翻转回A频段
	- P2收到最后一个点回复之后，通知P1开始测距
	- 假设P4先结束测距，翻转回A频段，广播自己已经测距完成消息
	- P2，P3记录下P4结束消息
	- P1结束，广播自己已经测距完成消息
	- P2，P3记录下P1结束消息
	- P2向第三次测距的两个发起者发送指令，开始第三次广播测距
	
	**Interface**
	u8 SendMsg(u8 *pointer, u8 arrayLenth), return IF_SUCCESS_SENDING(0x00 - T, 0xFF - F)

##第三次广播测距 - 0x0C
	- Pxl计算自己的两个镜像位置
	
	- Pxl发起第三次广播测距
	Cmtype | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
		   | CoodinateY | Dist1 | Dist2 | FLAG(RESV) | CRC | CRC

	0x0C | SenderID | LBD | UBD | X1 | Y1 | X2 | Y2 | FLAG | 0x0D | 0x0A
		   
	- Px回复
	Cmdtype | SenderID | TargetID | CRC | CRC
	0xC1 | SenderID | TargetID | 0x0D | 0x0A

	- Pxl广播
	Cmdtype | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - n times
	0xC2 | SenderID | TargetID | ts[4:1] | 0x0D | 0x0A

	- 最后一个节点回复
	Cmdtype | SenderID | TargetID | LocalizationStatus | dist[2:1] | CRC | CRC
	0xC3 | SenderID | TargetID | TorF | distwithPx[2:1] | 0x0D | 0x0A
	
	- 大家都计算出自己的坐标

	**Interface**
	- u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	- u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)