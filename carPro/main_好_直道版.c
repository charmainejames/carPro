/*===========================================================================
 * 存档版本：直道效果好 —— 加权差速 + 比例差速（允许内轮反转）
 * 备份时间：2026-06-05
 * 说明：用户确认此版直道效果好。这是基准巡线版，不含路口/直角弯/终点检测。
 *
 * 核心逻辑：
 *   - Track_GetError()：5路加权(-4~+4)算偏差，脱线沿用上次，全黑直行
 *   - 主循环：left=BASE+error*KP, right=BASE-error*KP，允许为负(内轮反转)
 *   - Floor 启动地板防死区；起步全速冲刺 120ms
 *
 * 关键参数（直道好的整定值）：
 *   BASE_SPEED=68  KP=26  MAX_SPEED=100  MIN_DRIVE=50
 *   TURN_HARD=1  DIR_SIGN=+1  LOST_TURN=40  LINE_LEVEL=0
 *
 * 恢复方法：把本文件内容覆盖回 main.c（去掉本注释块即可，或直接整体替换）。
 *===========================================================================*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"

sbit AIN1=P4^5;
sbit AIN2=P2^7;
sbit BIN1=P2^5;
sbit BIN2=P2^6;
sbit zuo2=P0^3;
sbit zuo1=P0^4;
sbit zhong=P0^2;
sbit you1=P0^1;
sbit you2=P0^0;
sbit LED1=P4^1;
sbit LED2=P4^2;
sbit LED3=P4^4;

PWMx_Duty PWMB_Duty;
u16 PWM_Period = 2000;

/*************  灰度巡线可调参数（上车整定看这里） **************/
#define BASE_SPEED   68     // 直道基础速度 0~100（弯道吃力就再降，太冲来不及转）
#define MAX_SPEED    100    // 单轮速度上限
#define MIN_DRIVE    50     // 最小启动速度：低于此值电机带不动整车
#define KP           26     // 转向比例系数：越大转向越猛（转弯不灵敏就加大）
#define TURN_HARD    1      // 急转弯时是否让内轮反转(原地差速转)，0=只减速，1=可倒转更猛
#define DIR_SIGN     (+1)   // 若发现“越修越偏/反向追线”，改成 (-1) 一键纠正左右
#define LOST_TURN    40     // 全白脱线时找线的转向差速（不要太大，否则转圈）

#define LINE_LEVEL   0      // 压在黑线上时，传感器引脚读到的电平

#define ON_LINE(pin)  ((pin) == LINE_LEVEL)   // 该传感器是否压在黑线上

int last_error = 0;         // 记忆：最后一次有效偏差（用于全白脱线找回）

/************************ IO初始化 ****************************/
void	GPIO_config(void)
{

}

/************************ 定时器初始化 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;
	TIM_InitStructure.TIM_ClkOut    = DISABLE;
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));
	TIM_InitStructure.TIM_PS        = 0;
	TIM_InitStructure.TIM_Run       = ENABLE;
	Timer_Inilize(Timer0,&TIM_InitStructure);
	NVIC_Timer0_Init(ENABLE,Priority_0);
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;
	PWM_Configuration(PWM5, &PWMx_InitStructure);

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;
	PWM_Configuration(PWM6, &PWMx_InitStructure);

	PWMx_InitStructure.PWM_Period   = PWM_Period;
	PWMx_InitStructure.PWM_DeadTime = 0;
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;
	PWM_Configuration(PWMB, &PWMx_InitStructure);

	PWM6_USE_P21();
	PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

void PWM_Left(int pwm)
{
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);
}

void PWM_Right(int pwm)
{
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);
}

void PWM_Run(int pwm1,int pwm2)
{
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

// 带方向的差速驱动：left/right 取值 -100~100，正=前进，负=反转。
// 实测左右接反，已对调：逻辑左轮=电机A(AIN/PWM5)，逻辑右轮=电机B(BIN/PWM6)。
void Motor_Drive(int left, int right)
{
	if (left >= 0) { AIN1=0; AIN2=1; }
	else           { AIN1=1; AIN2=0; left=-left; }
	if (right >= 0){ BIN1=1; BIN2=0; }
	else           { BIN1=0; BIN2=1; right=-right; }

	PWM_Run(right, left);
}

/*************  灰度巡线辅助函数  **************/

// 限幅：把速度约束到 [0, MAX_SPEED]
int Clamp(int v)
{
	if (v < 0)          return 0;
	if (v > MAX_SPEED)  return MAX_SPEED;
	return v;
}

// 启动地板：电机有死区，占空比太低只会嗡嗡不转。
int Floor(int v)
{
	if (v > 0 && v < MIN_DRIVE)  return MIN_DRIVE;
	return v;
}

// 计算线偏差。-4(线在最左) ~ +4(线在最右)，0=居中。压黑线=0。
int Track_GetError(void)
{
	int s_l1, s_l2, s_m, s_r1, s_r2;
	int cnt, sum;

	s_l1 = ON_LINE(zuo1);
	s_l2 = ON_LINE(zuo2);
	s_m  = ON_LINE(zhong);
	s_r1 = ON_LINE(you1);
	s_r2 = ON_LINE(you2);

	cnt = s_l1 + s_l2 + s_m + s_r1 + s_r2;
	sum = s_l1*(-4) + s_l2*(-2) + s_m*0 + s_r1*(+2) + s_r2*(+4);

	if (cnt == 0)
		return last_error;

	if (cnt >= 5)
	{
		last_error = 0;
		return 0;
	}

	last_error = (sum / cnt) * DIR_SIGN;
	return last_error;
}

/******************** 主函数 **************************/
void main(void)
{	int i;
	WTST = 0;
	EAXSFR();
	CKCON = 0;

	GPIO_config();
	Timer_config();
	PWM_config();

  P0M0=0xe0;
	P0M1=0x1f;
	P1M1=0x00;
	P1M0=0x00;
	P2M1=0x00;
	P2M0=0xFF;
  P4M0 = 0x16;
	P4M1 = 0x00;
	EA = 1;

	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;
	LED1=1;
	for(i=0; i<5; i++)
	{
		LED1 = 1;		delay_ms(130);		LED1 = 0;
		LED2 = 1;		delay_ms(130);		LED2 = 0;
		LED3 = 1;		delay_ms(130);		LED3 = 0;
	}

	// 启动冲刺：干电池带载起步难，先全速冲一下冲破静摩擦
	PWM_Run(100, 100);
	delay_ms(120);

	while (1)
	{
		int error, turn, left, right, cnt;

		cnt = ON_LINE(zuo1) + ON_LINE(zuo2) + ON_LINE(zhong)
		    + ON_LINE(you1) + ON_LINE(you2);

		if (cnt == 0)
		{
			// 全部脱线：朝最后偏离方向找线，不打死转圈
			if (last_error > 0)
				Motor_Drive(BASE_SPEED, -LOST_TURN);
			else if (last_error < 0)
				Motor_Drive(-LOST_TURN, BASE_SPEED);
			else
				Motor_Drive(BASE_SPEED, BASE_SPEED);
			continue;
		}

		// 正常巡线：加权偏差 + 差速（允许内轮反转）
		error = Track_GetError();
		turn  = error * KP;
		left  = BASE_SPEED + turn;
		right = BASE_SPEED - turn;
		if (left  > MAX_SPEED) left  = MAX_SPEED;
		if (right > MAX_SPEED) right = MAX_SPEED;
		if (!TURN_HARD) { if(left<0) left=0; if(right<0) right=0; }
		left  = Floor(left);
		right = Floor(right);
		Motor_Drive(left, right);
	}
}
