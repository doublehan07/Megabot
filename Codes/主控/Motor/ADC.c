#include "adc.h"

volatile uint16_t ADCValue[3]={0};

void ADC1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//clock initial
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//PC4 PC5
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

//ADC通用配置
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;  //独立采样
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//GPIO Initial
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	//ADC mode initial
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = 3;  //通道数
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1,&ADC_InitStructure);

	ADC_Cmd(ADC1,ENABLE);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_14,1,ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_15,2,ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_17,3,ADC_SampleTime_144Cycles);

	ADC_TempSensorVrefintCmd(ENABLE);
	//enable dma
	ADC_DMACmd(ADC1,ENABLE);
	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE); //源数据变化时开启DMA传输

	//enable ADC
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_SoftwareStartConv(ADC1);

}


void ADC1_DMAInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

	//DMA初始化
	DMA_InitStructure.DMA_BufferSize = 3;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADCValue;   //目标数据位
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));
	DMA_InitStructure.DMA_PeripheralBurst =DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0,ENABLE);

	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC, ENABLE);//
	
	
	//initialize interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void ADC_Init_User(void)
{
	ADC1_DMAInit();
	ADC1_Init();
}



//-------------------------------------------------------------------------------------

uint32_t ADC_Value_SMD[3]={0}; 

#define SMOOTHING_LEVEL   10
uint8_t ADC_Smoothing(void)
{
	static uint16_t count;
	uint8_t channel;
	if(count==0)
	{
		for(channel=0;channel<3;channel++)
			ADC_Value_SMD[channel]=0;
	}
	
	if((count++)<SMOOTHING_LEVEL)
	{
		for(channel=0;channel<3;channel++)
			ADC_Value_SMD[channel]+=ADCValue[channel];
		return 1;
	}
	else
	{
		count=0;
		for(channel=0;channel<3;channel++)
			ADC_Value_SMD[channel]=(ADC_Value_SMD[channel])/SMOOTHING_LEVEL;
		return 0;
	}
}


volatile uint16_t ADC_Voltage[3];
#define REF_ADC				ADC_Value_SMD[ADC_VOLREF]
#define REF_VOL				1200
void ADC_Calculate_Voltage(void)
{
	uint8_t channel;
	for(channel=0;channel<2;channel++)
		ADC_Voltage[channel]=((uint32_t)ADC_Value_SMD[channel]*REF_VOL)/REF_ADC;
	ADC_Voltage[ADC_VOLREF]=(4096*REF_VOL)/REF_ADC;
}


void DMA2_Stream0_IRQHandler(void)
{
	DMA_ClearFlag(DMA2_Stream0, DMA_IT_TCIF0);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
	
	if(ADC_Smoothing()==0)
	{
		ADC_Calculate_Voltage();
	}
	
}
