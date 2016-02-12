
#include "usart_dma.h"

static uint8_t txDMABuffer[64]; // DMA buffer 
static uint8_t rxDMABuffer[64];
static uint16_t txDMARear = sizeof(txDMABuffer); 

static uint8_t USART_RevBuf[64]; // 串口接收buffer 
static uint8_t USART_SndBuf[64]; // 串口发送buffer 

static bool Flag_Tx_Busy = 0;

//struct _Gsm_Rx{};
/*********************************************************************
*							初始化串口
**********************************************************************/

void init_uart(void)
{
	//定义中断结构体
	NVIC_InitTypeDef NVIC_InitStructure ;
	//定义IO初始化结构体
 	GPIO_InitTypeDef GPIO_InitStructure;
	//定义串口结构体  
	USART_InitTypeDef USART_InitStructure;
	//定义DMA结构体
	DMA_InitTypeDef DMA_InitStructure;

	//打开串口对应的外设时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	 //串口发DMA配置  
	//启动DMA时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//DMA发送中断设置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	


	//DMA通道配置
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4; 
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr =  (uint32_t)txDMABuffer;
	//dma传输方向
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = sizeof(txDMABuffer);
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	
	//指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    
	//指定了FIFO阈值水平
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;        
	//指定的Burst转移配置内存传输 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       
	//指定的Burst转移配置外围转移 */  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 

	//配置DMA1的通道         
	DMA_Init(DMA1_Stream6, &DMA_InitStructure);  
	//使能中断
	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);   



	//串口收DMA配置  
	//启动DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	//DMA通道配置
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	//外设地址
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	//内存地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rxDMABuffer;
	//dma传输方向
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	//设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_BufferSize = sizeof(rxDMABuffer);
	//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//设置DMA的内存递增模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//外设数据字长
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	//内存数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	//设置DMA的传输模式
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	//设置DMA的优先级别
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	
	//指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式  
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;    
	//指定了FIFO阈值水平
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;        
	//指定的Burst转移配置内存传输 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;       
	//指定的Burst转移配置外围转移 */  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	
	//配置DMA1的通道         
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);  
	//使能通道
	DMA_Cmd(DMA1_Stream5,ENABLE);
	
    //初始化串口参数  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = 115200; 
	//初始化串口 
    USART_Init(USART2,&USART_InitStructure);  
	
	//中断配置
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  

	//配置中断  
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;               //通道设置为串口中断  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //中断占先等级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //中断响应优先级 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
    NVIC_Init(&NVIC_InitStructure);   
        
	//采用DMA方式发送
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
	//采用DMA方式接收
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);

	//中断配置
	USART_ITConfig(USART2,USART_IT_TC,DISABLE);
	USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  
    //启动串口  
  USART_Cmd(USART2, ENABLE);    

    //设置IO口时钟      
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);  
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

    //管脚模式:输出口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
    //类型:推挽模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
    //上拉下拉设置
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	
	//IO口速度
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    //管脚指定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    //初始化
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    //管脚模式:输入口
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
    //上拉下拉设置
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
    //管脚指定
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    //初始化
	GPIO_Init(GPIOA, &GPIO_InitStructure);      
}

/*********************************************************************
*							接口函数:向gsm模块发送数据
*参数:data:发送数据存放地址
*	  size:发送数据字节数
**********************************************************************/

void drv_tx(uint8_t *data,uint16_t size)
{
	//等待空闲
	while (Flag_Tx_Busy);
	Flag_Tx_Busy = 1;
	//复制数据
	memcpy(txDMABuffer,data,size);
	//设置传输数据长度
	DMA_SetCurrDataCounter(DMA1_Stream6,size);
	//打开DMA,开始发送
 	DMA_Cmd(DMA1_Stream6,ENABLE);
}


/*********************************************************************
*							接口函数:DMA发送中断处理函数
**********************************************************************/

void drv_gsm_deal_irq_dma_tx(void)
{
	if(DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6) != RESET) 
	{
		//清除标志位
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
		//关闭DMA
		DMA_Cmd(DMA1_Stream6,DISABLE);
		//打开发送完成中断,发送最后两个字节
		USART_ITConfig(USART2,USART_IT_TC,ENABLE);
	}
}

/*********************************************************************
*							处理串口发送完成中断
*返回:0:未产生,1:已经产生
**********************************************************************/

uint8_t drv_gsm_deal_irq_tx_end(void)
{
	if(USART_GetITStatus(USART2, USART_IT_TC) != RESET)
    {
		//关闭发送完成中断
		USART_ITConfig(USART2,USART_IT_TC,DISABLE);
		//发送完成
        Flag_Tx_Busy = 0;
		
		return 1;
    } 
	
	return 0;
}

/*********************************************************************
*							处理串口接收完成中断
*参数:buf:接收的数据
*     len:接收的数据长度
*返回:0:未产生,其他:已经产生,此值为接收的数据长度
**********************************************************************/

uint8_t drv_gsm_deal_irq_rx_end(uint8_t *buf)
{	
	uint16_t len = 0;
	
	//接收完成中断
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
    	USART2->SR;
    	USART2->DR; //清USART_IT_IDLE标志
		//关闭DMA
    	DMA_Cmd(DMA1_Stream5,DISABLE);
		//清除标志位
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
		
		//获得接收帧帧长
		len = sizeof(rxDMABuffer) - DMA_GetCurrDataCounter(DMA1_Stream5);
		memcpy(buf,rxDMABuffer,len);
		
		//设置传输数据长度
		DMA_SetCurrDataCounter(DMA1_Stream5,sizeof(rxDMABuffer));
    	//打开DMA
		DMA_Cmd(DMA1_Stream5,ENABLE);

		return len;
    } 
	
	return 0;
}

/*********************************************************************
*							GSM模块:DMA发送中断处理函数
**********************************************************************/

void DMA1_Stream6_IRQHandler(void)
{
    gsm_dma_tx_irq_handler();
}
   
/*********************************************************************
*							GSM模块:串口中断处理函数
**********************************************************************/

void USART2_IRQHandler(void) 
{
	gsm_irq_handler();
}

/*********************************************************************
*							接口函数:DMA发送中断处理函数
**********************************************************************/

void gsm_dma_tx_irq_handler(void)
{
	inf_gsm_deal_irq_dma_tx();
}
   
/*********************************************************************
*							接口函数:串口中断处理函数
*参数:data:接收数据存放地址
*返回:接收数据长度
**********************************************************************/

void gsm_irq_handler(void)                              
{   
	// struct _Gsm_Rx rx;
	// uint8_t i = 0;
	
	//发送完成中断处理
	//uint8_t i = inf_gsm_deal_irq_tx_end();
	
	//接收完成中断处理
	//rx.len = inf_gsm_deal_irq_rx_end(rx.buf);
	// if (rx.len != 0)
	// {
	// 	//通知观察者
	// 	for (i = 0;i < Len_Observer;i++)
	// 	{
	// 		Observer[i](rx);
	// 	}
 //    } 
} 

/*********************************************************************
*							接口函数:DMA发送中断处理函数
**********************************************************************/

void inf_gsm_deal_irq_dma_tx(void)
{
	drv_gsm_deal_irq_dma_tx();
}

/*********************************************************************
*							处理发送完成中断
*返回:0:未产生,1:已经产生
**********************************************************************/

uint8_t inf_gsm_deal_irq_tx_end(void)
{
	return drv_gsm_deal_irq_tx_end();
}

/*********************************************************************
*							处理接收完成中断
*参数:buf:接收的数据
*     len:接收的数据长度
*返回:0:未产生,其他:已经产生,此值为接收的数据长度
**********************************************************************/

uint8_t inf_gsm_deal_irq_rx_end(uint8_t *buf)
{
	return drv_gsm_deal_irq_rx_end(buf);
}



