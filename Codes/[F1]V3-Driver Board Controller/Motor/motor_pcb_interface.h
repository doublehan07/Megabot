/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PCB_INTI_H
#define __MOTOR_PCB_INTI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
struct STime
{
	unsigned char ucYear;
	unsigned char ucMonth;
	unsigned char ucDay;
	unsigned char ucHour;
	unsigned char ucMinute;
	unsigned char ucSecond;
	unsigned short usMiliSecond;
};

struct SAcc
{
	short a[3];
	short T;
};

struct SGyro
{
	short w[3];
	short T;
};

struct SAngle
{
	short Angle[3];
	short T;
};

struct SMag
{
	short h[3];
	short T;
};

struct SDStatus
{
	short sDStatus[4];
};

struct SPress
{
	long lPressure;
	long lAltitude;
};

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};

struct SQuat
{
	short Q[4];
};

struct SSatePre
{
	short SN;
	short PDOP;
	short HDOP;
	short VDOP;
};

/* Exported constants --------------------------------------------------------*/
extern struct STime				stcTime;
extern struct SAcc 				stcAcc;
extern struct SGyro 			stcGyro;
extern struct SAngle 			stcAngle;
extern struct SMag 				stcMag;
extern struct SDStatus 		stcDStatus;
extern struct SPress 			stcPress;
extern struct SLonLat 		stcLonLat;
extern struct SGPSV 			stcGPSV;
extern float sAngle;

/* Exported macro ------------------------------------------------------------*/
//DRV8801
//Signal
/* left */
#define PHASE_PIN_LEFT								GPIO_Pin_4                                                                                                                      
#define ENABLE_PIN_LEFT							GPIO_Pin_6								//TIM3_CH1 - PA6
#define ENABLE_PWMO_LEFT 				CCR1

/* right */
#define PHASE_PIN_RIGHT							GPIO_Pin_5                                                                                                                    
#define ENABLE_PIN_RIGHT						GPIO_Pin_7								//TIM3_CH2 - PA7
#define ENABLE_PWMO_RIGHT	 		CCR2

#define PWM_TIM 											    TIM3
#define PWM_GPIO													GPIOA
#define SIGNAL_GPIO 								    GPIOA

//Control - PCx
/* left */
#define nFAULT_LEFT										GPIO_Pin_4
#define MODE1_LEFT										GPIO_Pin_5
#define MODE2_LEFT										GPIO_Pin_6
#define nSLEEP_LEFT										GPIO_Pin_7

/* right */
#define nFAULT_RIGHT									GPIO_Pin_8
#define MODE1_RIGHT									GPIO_Pin_9
#define MODE2_RIGHT									GPIO_Pin_10
#define nSLEEP_RIGHT									GPIO_Pin_11

#define CONTROL_GPIO								GPIOB

//ADC - PAx
/* left */
//#define Vpropi_LEFT										GPIO_Pin_1.
//#define ADC_GPIO_LEFT									GPIOA

///* right */
//#define Vpropi_RIGHT									GPIO_Pin_2
//#define ADC_GPIO_RIGHT								GPIOA

/* Exported functions ------------------------------------------------------- */

void Motor_Init(void);

void Motor_If_Awake(u8 if_awake);
void Motor_If_FastDecay(u8 if_fast_decay);
void Motor_If_Forward(u8 left_or_right, u8 if_forward);
void Motor_Set_Speed(u8 left_or_right, u16 speed);

void Usart_PID_init(void);
void Usart_JY901_init(void);
void ParseSerialData(unsigned char ucData);
int16_t Inertia_Get_Angle_Yaw(void);
void Encoder_EXTI_Configuration(void);

void Sampling_Tick_Speed(void);
u16 Get_Speed(u8 left_or_right);

// PID -- set the speed
int16_t MotorSpeedPID(int16_t speed);
void setSpeed(int16_t speed);
int16_t returnSpeed(void);

// PID -- turn the angle
void Motor_Move(int16_t angle, u8 if_related, int16_t speed);
int16_t returnAngle(void);

// data to be sent of PID and JY901
static char sendDataPID[5];
static char sendDataAngle[5];

#endif /* __MOTOR_PCB_INTI */
