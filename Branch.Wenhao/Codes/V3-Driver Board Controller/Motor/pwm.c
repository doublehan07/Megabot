/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_pcb_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint16_t PWM_Period = 1000; //PWM Max Value

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
//Motor Configuration
void Timer1_PWM_Configuration(void)
{
	//Set Timer1 as PWM output timer for motor controlling - ENABLE
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	//����ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	//��ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; 							//��ʱ��Ԥ��Ƶ APB2 /(TIM_Prescaler + 1) = 1M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = PWM_Period - 1; 					//�Զ���װֵ (TIM_Period + 1) = 1k
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//����ʱ�ӷָ�: TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	//��ʱ���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ģʽPWM1��С�ڱȽ�ֵʱ��Ч
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;		//TIM1CH1-TIM1CH4ʧ��
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;		//TIM1CH1N-TIM1CH4Nʹ��
	TIM_OCInitStructure.TIM_Pulse = PWM_Period;												//ÿ�β���ıȽ�ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;					//��Ч��ƽΪ�͵�ƽ������ʹ�õ��Ƿ��������
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; 			//���ͬ��,TIM_OCNPolarity_Highʱ�������
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;			//TIM1CH1-TIM1CH4���״̬
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;			//TIM1CH1N-TIM1CH4N���״̬

	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure); //TIM2N
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure); //TIM3N

	TIM_Cmd(PWM_TIM, ENABLE);
	TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
}

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//Usart1 init for Yaw
	Usart_JY901_init();
	
	//Encoder init for speed
	Encoder_EXTI_Configuration();
	
	//Enable GPIO used for controlling motor
	//Signal
	GPIO_InitStructure.GPIO_Pin = PHASE_PIN_LEFT | PHASE_PIN_LEFT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(SIGNAL_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENABLE_PIN_LEFT | ENABLE_PIN_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(SIGNAL_GPIO, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(SIGNAL_GPIO, ENABLE_Source_LEFT, ENABLE_TIM);
	GPIO_PinAFConfig(SIGNAL_GPIO, ENABLE_Source_RIGHT, ENABLE_TIM);
	
	Timer1_PWM_Configuration();
	
	//Control
	GPIO_InitStructure.GPIO_Pin = nFAULT_LEFT | MODE1_LEFT | MODE2_LEFT | nSLEEP_RIGHT \
															| nFAULT_RIGHT | MODE1_RIGHT | MODE1_RIGHT | nSLEEP_LEFT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(CONTROL_GPIO, &GPIO_InitStructure);
	
	//ADC
	
}
