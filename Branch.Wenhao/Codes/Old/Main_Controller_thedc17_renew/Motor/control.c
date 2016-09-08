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
#define ENCODER							50 //减速比
#define MAXSPEED						15
#define RANGE 							5
#define MOTOR_LEFT_F				GPIO_ResetBits(MOTOR_GPIO , PHASE_LEFT)
#define MOTOR_LEFT_B				GPIO_SetBits(MOTOR_GPIO , PHASE_LEFT)
#define MOTOR_RIGHT_F				GPIO_ResetBits(MOTOR_GPIO , PHASE_RIGHT)
#define MOTOR_RIGHT_B				GPIO_SetBits(MOTOR_GPIO , PHASE_RIGHT)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
double InputSpeed_L = 0.0;
double InputSpeed_R = 0.0;
__IO u16 InputSpeedCounter_L = 0;
__IO u16 InputSpeedCounter_R = 0;
__IO u8 InputSpeedPhase_L = 0;
__IO u8 InputSpeedPhase_R = 0;
u16 Encoder_Counter_L = 0;
u16 Encoder_Counter_R = 0;
__IO u16 LastCycleCounter_L = 0;
__IO u16 LastCycleCounter_R = 0;

double InputSpeed[2] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	Encoder_Counter_L++;
	Encoder_Counter_R++;
	
	//防止溢出
	if(Encoder_Counter_L > 1000)
		Encoder_Counter_L = 0;
	if(Encoder_Counter_R > 1000)
		Encoder_Counter_R = 0;
}

void EXTI0_IRQHandler(void) //左编码器
{
	InputSpeedPhase_L = (GPIO_ReadInputData(GPIOA) & 0x03); //0000 0011
  EXTI_ClearFlag(EXTI_Line0);
  EXTI_ClearITPendingBit(EXTI_Line0);
	InputSpeedCounter_L++;
	LastCycleCounter_L = Encoder_Counter_L;
	Encoder_Counter_L = 0;
	
	if (state[0]^(InputSpeedPhase_L&0x01))
	{
		if (InputSpeedPhase_L == 0x00 || InputSpeedPhase_L == 0x03)
			distance[0]++;
		else
			distance[0]--;
		state[0]=InputSpeedPhase_L&0x01;
		count[0]++;
	}
	count_t[0]++;
}

void EXTI2_IRQHandler(void) //右编码器
{
	InputSpeedPhase_R = (GPIO_ReadInputData(GPIOA) & 0x0C); //0000 1100
  EXTI_ClearFlag(EXTI_Line2);
  EXTI_ClearITPendingBit(EXTI_Line2);
	InputSpeedCounter_R++;
	LastCycleCounter_R = Encoder_Counter_R;
	Encoder_Counter_R = 0;
		
	if (state[1]^(InputSpeedPhase_R&0x04))
	{
		if (InputSpeedPhase_R == 0x00 || InputSpeedPhase_R == 0x0c)
			distance[1]++;
		else
			distance[1]--;
		state[1]=InputSpeedPhase_R&0x04;
		count[1]++;
	}
	count_t[1]++;
}

void Sampling_Tick_Speed(void) //每50ms被调用一次,额定转速折算下来是15次计数
{
	//Phase == 1 => B >>> A
	static double Head_L = 0.0, Head_R = 0.0, Save_L = 0.0, Save_R = 0.0;
	
	//防止除数为0
	if(LastCycleCounter_L == 0)
		LastCycleCounter_L = 10;
	if(LastCycleCounter_R == 0)
		LastCycleCounter_R = 10;
	
	Head_L = Encoder_Counter_L / (double)LastCycleCounter_L; //计算当前比例值
	Head_R = Encoder_Counter_R / (double)LastCycleCounter_R; 
	
	InputSpeed[0] = InputSpeedCounter_L + Head_L + Save_L;
	InputSpeed[1] = InputSpeedCounter_R + Head_R + Save_R;
	
	InputSpeedCounter_L = 0;
	InputSpeedCounter_R = 0;
	
	InputSpeed_L = InputSpeedPhase_L ? -InputSpeed_L: InputSpeed_L;
	InputSpeed_R = InputSpeedPhase_R ? -InputSpeed_R: InputSpeed_R;
	
	//更新下一次开头的比例值
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
		counter = 0; //每50ms采样一次
		Sampling_Tick_Speed();
	}
	ADC_PWM_Motor_Exec();
}
