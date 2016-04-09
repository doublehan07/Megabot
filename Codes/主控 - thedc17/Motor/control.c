/**
  ******************************************************************************
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "port.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ENCODER							50 //���ٱ�
#define MAXSPEED						15
#define RANGE 							5
#define MOTOR_LEFT_F				GPIO_ResetBits(MOTOR_GPIO , PHASE_LEFT)
#define MOTOR_LEFT_B				GPIO_SetBits(MOTOR_GPIO , PHASE_LEFT)
#define MOTOR_RIGHT_F				GPIO_ResetBits(MOTOR_GPIO , PHASE_RIGHT)
#define MOTOR_RIGHT_B				GPIO_SetBits(MOTOR_GPIO , PHASE_RIGHT)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO double InputSpeed_L = 0.0;
__IO double InputSpeed_R = 0.0;
__IO u16 InputSpeedCounter_L = 0;
__IO u16 InputSpeedCounter_R = 0;
__IO u8 InputSpeedPhase_L = 0;
__IO u8 InputSpeedPhase_R = 0;
u16 Encoder_Counter_L = 0;
u16 Encoder_Counter_R = 0;
__IO u16 LastCycleCounter_L = 0;
__IO u16 LastCycleCounter_R = 0;

u8 MOTOR_RUNNING_FLAG = 0; //����ָʾflag
u8 MOTOR_TURNNING_FLAG = 0;

int16_t motorSpeed[2] = {0};
double InputSpeed[2] = {0};

u8 dirFlag = 0; //0-˳ʱ�� 1-��ʱ��
u16 setAngle = 0;

const double P_Speed = 0.06;
const double I_Speed = 0;
const double D_Speed = 0;

const double P_Angle = 0.1;
const double I_Angle = 0.0;
const double D_Angle = 0.0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Motor_Setspeed(int16_t speedL, int16_t speedR)
{
	motorSpeed[0] = speedL;
	motorSpeed[1] = speedR;
	MOTOR_RUNNING_FLAG = 10;
}

void Motor_Set(u16 val)
{
	PWM_TIM -> ENABLE_LEFT_PWMO = val;
	PWM_TIM -> ENABLE_RIGHT_PWMO = val;
}

void Motor_Speed_PID(void)
{
	static u8 phaseFlag = 1; //����ָʾ,1ǰ0��
	static u8 i = 0;
	static int16_t speed = 0;
	
	//����ʽPID����
	static double PauseCounterSet = 0.0;
	static double error = 0.0, last1Error = 0.0, last2Error = 0.0;
	static double PVAL = 0.0, IVAL = 0.0, DVAL = 0.0;
	static double outSpeed = 0.0;
	static u16 ccrVal = 0;
	
	if(MOTOR_RUNNING_FLAG)
		MOTOR_RUNNING_FLAG--;
	
	for(i = 0; i < 2; i++)
	{
		speed = motorSpeed[i];
		if(speed < 0)
		{
			phaseFlag = 0;
			speed = -speed;
		}	
		speed = (speed>100) ? 100: speed;	//�ٶ�threshold: 100
		PauseCounterSet = (double)speed * MAXSPEED / 100; //���ٶ�ת��Ϊ������
		
		//�������
		error = PauseCounterSet - InputSpeed[i];
		if(error < RANGE)
			continue;
			
		PVAL = error - last1Error;
		IVAL = (error + last1Error) / 2;
		DVAL = error - 2 * last1Error + last2Error;
			
		outSpeed = speed + P_Speed * PVAL + I_Speed * IVAL + I_Speed * DVAL;
		ccrVal = (u16)(outSpeed * 10);
		ccrVal = (ccrVal>1000) ? 1000: ccrVal;
			
		last2Error = last1Error;
		last1Error = error;
			
		if(phaseFlag)
		{
			if(i)
				MOTOR_RIGHT_F;
			else
				MOTOR_LEFT_F;
		}
		else
		{
			if(i)
				MOTOR_RIGHT_B;
			else
				MOTOR_LEFT_B;
		}
		
		if(i)
		{
			PWM_TIM -> ENABLE_RIGHT_PWMO = ccrVal;
		}
		else
		{
			PWM_TIM -> ENABLE_LEFT_PWMO = ccrVal;
		}
	}
}

void Motor_SetAngle(int16_t angle, u8 isRelative) //��ʱ������˳ʱ���С
{
	setAngle = isRelative ? ((sAngle+angle)%360): sAngle;
	MOTOR_TURNNING_FLAG = 10;
}

void Motor_Turnangle(void) //��ʱ������˳ʱ���С
{	
	//����ʽPID����
	static double error = 0.0, last1Error = 0.0, last2Error = 0.0;
	static double PVAL = 0.0, IVAL = 0.0, DVAL = 0.0;
	static double dA = 0.0;
	static u16 dAngle = 0;
	static u16 dirError = 0;
	static u16 deltaSpeed;
	
	dA = setAngle - sAngle;
	if(dA < RANGE || dA > 360-RANGE)
	{
		deltaSpeed = 0;
		if (MOTOR_TURNNING_FLAG)
			MOTOR_TURNNING_FLAG--;
	}
	else
	{
		//�������ͷ���
		dirError = (setAngle - sAngle + 720) % 360;
		if(dirError > 180) //��ʱ��ת360-ans
		{
			dirFlag = 1;
			error = 360 - dirError;
		}
		else //˳ʱ��תans
		{
			dirFlag = 0;
			error = dirError;
		}
		
		PVAL = error - last1Error;
		IVAL = (error + last1Error) / 2;
		DVAL = (error - 2 * last1Error + last2Error);
		
		//PID�����ֱ�ӶԲ���޷�����
		dAngle = setAngle - 10 + P_Angle * PVAL + I_Angle * IVAL + P_Angle * DVAL;
		dAngle = (dAngle>100) ? 100: dAngle;
		
		deltaSpeed = dAngle / 2; //dAngle / 200 * 100
	
		if(dirFlag) //��ʱ��
		{
			Motor_Setspeed(MOTOR_LEFT, -deltaSpeed);
			Motor_Setspeed(MOTOR_RIGHT, deltaSpeed);
		}
		else //˳ʱ��
		{
			Motor_Setspeed(MOTOR_LEFT, deltaSpeed);
			Motor_Setspeed(MOTOR_RIGHT, -deltaSpeed);
		}
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	Encoder_Counter_L++;
	Encoder_Counter_R++;
	
	//��ֹ���
	if(Encoder_Counter_L > 1000)
		Encoder_Counter_L = 0;
	if(Encoder_Counter_R > 1000)
		Encoder_Counter_R = 0;
}

void EXTI0_IRQHandler(void) //�������
{
	InputSpeedPhase_L = (GPIO_ReadInputData(GPIOA) & 0x03); //0000 0011
  EXTI_ClearFlag(EXTI_Line0);
  EXTI_ClearITPendingBit(EXTI_Line0);
	InputSpeedCounter_L++;
	LastCycleCounter_L = Encoder_Counter_L;
	Encoder_Counter_L = 0;
}

void EXTI2_IRQHandler(void) //�ұ�����
{
	InputSpeedPhase_R = (GPIO_ReadInputData(GPIOA) & 0x0C); //0000 1100
  EXTI_ClearFlag(EXTI_Line2);
  EXTI_ClearITPendingBit(EXTI_Line2);
	InputSpeedCounter_R++;
	LastCycleCounter_R = Encoder_Counter_R;
	Encoder_Counter_R = 0;
}

void Sampling_Tick_Speed(void) //ÿ50ms������һ��,�ת������������15�μ���
{
	//Phase == 1 => B >>> A
	static double Head_L = 0.0, Head_R = 0.0, Save_L = 0.0, Save_R = 0.0;
	
	//��ֹ����Ϊ0
	if(LastCycleCounter_L == 0)
		LastCycleCounter_L = 10;
	if(LastCycleCounter_R == 0)
		LastCycleCounter_R = 10;
	
	Head_L = Encoder_Counter_L / (double)LastCycleCounter_L; //���㵱ǰ����ֵ
	Head_R = Encoder_Counter_R / (double)LastCycleCounter_R; 
	
	InputSpeed[0] = InputSpeedCounter_L + Head_L + Save_L;
	InputSpeed[1] = InputSpeedCounter_R + Head_R + Save_R;
	
	InputSpeedCounter_L = 0;
	InputSpeedCounter_R = 0;
	
	InputSpeed_L = InputSpeedPhase_L ? -InputSpeed_L: InputSpeed_L;
	InputSpeed_R = InputSpeedPhase_R ? -InputSpeed_R: InputSpeed_R;
	
	//������һ�ο�ͷ�ı���ֵ
	Save_L = 1 - Head_L;
	Save_R = 1 - Head_R;
}

void SysTick_Handler(void)
{		
	static u8 counter = 0;
	TimingDelay_Decrement();
	counter++;
	if(counter >= 5)
	{
		counter = 0; //ÿ50ms����һ��
		Sampling_Tick_Speed();
		Motor_Speed_PID();
	}
	Motor_Turnangle();
}
