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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	//��ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1; 							//��ʱ��Ԥ��Ƶ APB2 /(TIM_Prescaler + 1) = 1M
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period = PWM_Period - 1; 					//�Զ���װֵ (TIM_Period + 1) = 1k
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 		//����ʱ�ӷָ�: TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(PWM_TIM,&TIM_TimeBaseStructure);

	//��ʱ���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ģʽPWM1��С�ڱȽ�ֵʱ��Ч
		
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//TIM1CH1-TIM1CH4ʧ��
	TIM_OCInitStructure.TIM_Pulse = PWM_Period;												//ÿ�β���ıȽ�ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;					//��Ч��ƽΪ�͵�ƽ������ʹ�õ��Ƿ��������
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;			//TIM1CH1-TIM1CH4���״̬
 
	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure); //CH1
	TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
		
	TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure); //CH2
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(PWM_TIM, ENABLE);

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
	GPIO_InitStructure.GPIO_Pin = PHASE_PIN_LEFT | PHASE_PIN_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SIGNAL_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENABLE_PIN_LEFT | ENABLE_PIN_RIGHT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(PWM_GPIO, &GPIO_InitStructure);
	
	Timer1_PWM_Configuration();
	
	//Control
	GPIO_InitStructure.GPIO_Pin = nFAULT_LEFT   | MODE1_LEFT   | MODE2_LEFT   | nSLEEP_RIGHT \
															                      | nFAULT_RIGHT | MODE1_RIGHT | MODE1_RIGHT | nSLEEP_LEFT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CONTROL_GPIO, &GPIO_InitStructure);
	
	//ADC
	//TODO
}
