#ͨ������
##�ĸ��ӽڵ�Ķ�λ
	- P1(Leader)�㲥������������
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC
	0x0A | 0x01 | 0x02 | 0x04 | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 | 0x0D | 0x0A

	- P2-P4�ظ�
	Cmdtype | SenderID | TargetID | CRC | CRC
	0x1A | SenderID } 0x01 | 0x0D | 0x0A

	- P1�㲥
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 3 times
	0x2A | 0x01 | TargetID | ts[4:1] | 0x0D | 0x0A

	//ע��У�飬���ﲻҪ���һ�λظ�

	- P2�㲥
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC
	0x0A | 0x02 | 0x03 | 0x04 | X | 0x00 | 0x00 | 0x00 | 0x00 | 0x0D | 0x0A

	- P3-P4�ظ�
	CmdType | SenderID | TargetID | CRC | CRC
	0x1A | SenderID | 0x02 | 0x0D | 0x0A

	- P2�㲥
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 2 times
	0x2A | 0x02 | TargetID | ts[4:1] | 0x0D | 0x0A

	//ע��У�飬���ﲻҪ���һ�λظ�

	- P3�㲥
	CmdType | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
			| CoodinateY | Dist1 | Dist2 | FLAG(RESV) | CRC | CRC
	0x0A | 0x03 | 0x04 | 0x04 | X1 | Y1 | X2 | Y2 | 0x00 | 0x0D | 0x0A

	- P4�ظ�
	CmdType | SenderID | TargetID | CRC | CRC
	0x1A | 0x04 | 0x03 | 0x0D | 0x0A

	- P3�㲥
	CmdType | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - 2 times
	0x2A | 0x03 | 0x04 | ts[4:1] | 0x0D | 0x0A

	//ע��У�飬���ﲻҪ���һ�λظ�

	**Interface**
	u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)

##�Ľڵ�ĺ�������
	- P4������Լ���P3�����꣬ͬʱ֪��P1��P2������
	- P4��ȫ����N���ڵ�㲥P1-P4������Ϣ
	- P4����P2,P3��ʼ���
	- P3���߽ڵ��Ƿ�ת��BƵ��
	**Interface**
	u8 SendMsg(u8 *pointer, u8 arrayLenth), return IF_SUCCESS_SENDING(0x00 - T, 0xFF - F)

##�ڶ��ι㲥��� - 0x0B
	- P2����ڶ��ι㲥���
	Cmtype | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
		   | CoodinateY | Dist1(RESV) | Dist2(RESV) | FLAG(RESV) | CRC | CRC

	0x0B | SenderID | LBD | UBD | X | Y | D1 | D2 | FLAG | 0x0D | 0x0A
		   
	- Px�ظ�
	Cmdtype | SenderID | TargetID | CRC | CRC
	0xB1 | SenderID | TargetID | 0x0D | 0x0A

	- P2�㲥
	Cmdtype | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - n times
	0xB2 | SenderID | TargetID | ts[4:1] | 0x0D | 0x0A

	- ���һ���ڵ�ظ�
	Cmdtype | SenderID | TargetID | LocalizationStatus | dist[2:1] | CRC | CRC
	0xB3 | SenderID | TargetID | TorF | distwithP2[2:1] | 0x0D | 0x0A

	**Interface**
	- u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	- u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)
	
##�´ι㲥���֮ǰ��ͬ������
	- P3�յ����һ����ظ�֮��֪ͨP4��ʼ��࣬ͬʱ��ת��AƵ��
	- P2�յ����һ����ظ�֮��֪ͨP1��ʼ���
	- ����P4�Ƚ�����࣬��ת��AƵ�Σ��㲥�Լ��Ѿ���������Ϣ
	- P2��P3��¼��P4������Ϣ
	- P1�������㲥�Լ��Ѿ���������Ϣ
	- P2��P3��¼��P1������Ϣ
	- P2������β������������߷���ָ���ʼ�����ι㲥���
	
	**Interface**
	u8 SendMsg(u8 *pointer, u8 arrayLenth), return IF_SUCCESS_SENDING(0x00 - T, 0xFF - F)

##�����ι㲥��� - 0x0C
	- Pxl�����Լ�����������λ��
	
	- Pxl��������ι㲥���
	Cmtype | SenderID | RecieveIDLowerBound | RecieveIDUpperBound | CoodinateX \
		   | CoodinateY | Dist1 | Dist2 | FLAG(RESV) | CRC | CRC

	0x0C | SenderID | LBD | UBD | X1 | Y1 | X2 | Y2 | FLAG | 0x0D | 0x0A
		   
	- Px�ظ�
	Cmdtype | SenderID | TargetID | CRC | CRC
	0xC1 | SenderID | TargetID | 0x0D | 0x0A

	- Pxl�㲥
	Cmdtype | SenderID | TargetID | TimeStamp[4:1] | CRC | CRC - n times
	0xC2 | SenderID | TargetID | ts[4:1] | 0x0D | 0x0A

	- ���һ���ڵ�ظ�
	Cmdtype | SenderID | TargetID | LocalizationStatus | dist[2:1] | CRC | CRC
	0xC3 | SenderID | TargetID | TorF | distwithPx[2:1] | 0x0D | 0x0A
	
	- ��Ҷ�������Լ�������

	**Interface**
	- u8 Initiator_Ranging(), return IF_SUCCESS_RANGING(0x00 - T, 0xFF - F)
	- u16 Receptor_Ranging(), return IF_GET_DIST(distance_cm, 0xFFFF - F)