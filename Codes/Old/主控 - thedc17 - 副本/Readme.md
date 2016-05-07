#时钟配置

HSE = 8MHz

HCLK(AHB） = 168MHz, PCLK2(APB2) = 84MHz, PCLK1(APB1) = 42MHz 	

PLLCLK = HSE(8M) / 8 * 336 / 2 = 168MHz，PLL_Q = 7

Select PLL as system clock source

Systick = 1ms

#电机
- 比较容易修改的内容都定义在MovementCtr.h中
- 目前使用2节18650，速度不是很快
- 外部调用都仅需要包含MovementCtr.h
- PWM的设置比较多，目前是用的TIM3在PA6和PA7，这个需要到PWM.c中修改
#接口：
- MOTOR_TURNING_FLAG，转向标志位；
- ANGLE_INPUT，绝对方向的输入角度（单位为度），同时也是转向时的重要参数（目前由JY901.c生成）
- P和D，直接乘在角度（度）和负的角速度（度/秒）前面的，结果单位是百分比
- GoAndTurn(short angle, uint8_t isRelative, int16_t speed)，角度单位度，速度单位百分比（±100）
- ADC_PWM_Motor_Init()，初始化
- ADC_PWM_Motor_Exec()，在SysTick中执行，1~10ms一次，进行速度调整