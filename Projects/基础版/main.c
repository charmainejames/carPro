#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
/*************	����˵��	**************
�߼�PWM��ʱ�� PWM5,PWM6,PWM7,PWM8 ÿ��ͨ�����ɶ���ʵ��PWM���.
4��ͨ��PWM������Ҫ���ö�Ӧ����ڣ���ͨ��ʾ�����۲�������ź�.
PWM���ں�ռ�ձȿ����Զ������ã���߿ɴ�65535.
����ʱ, ѡ��ʱ�� 24MHZ (�û�����"config.h"�޸�Ƶ��).
******************************************/
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
#define KP           32     // 转向比例系数：越大转向越猛（偏差稍大内轮就反转，原地差速急转）
#define TURN_HARD    1      // 急转弯时是否让内轮反转(原地差速转)，0=只减速，1=可倒转更猛
#define DIR_SIGN     (+1)   // 若发现“越修越偏/反向追线”，改成 (-1) 一键纠正左右
#define LOST_TURN    40     // 全白脱线时找线的转向差速（不要太大，否则转圈）

// 黑线对应的电平：你的模块“压黑线 = 灯暗”。多数情况下压黑线输出低电平=0。
// 若烧进去发现行为全反（追白不追黑/乱转），把这里改成 1 即可。
#define LINE_LEVEL   0      // 压在黑线上时，传感器引脚读到的电平

#define ON_LINE(pin)  ((pin) == LINE_LEVEL)   // 该传感器是否压在黑线上

int last_error = 0;         // 记忆：最后一次有效偏差（用于全白脱线找回）

/*************	���غ�������	**************/

/*************  �ⲿ�����ͱ������� *****************/

/************************ IO������ ****************************/
void	GPIO_config(void)
{
	
}

/************************ ��ʱ������ ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//�ṹ����
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//�ж�Ƶ��, 1000��/��
	TIM_InitStructure.TIM_PS        = 0;					//8λԤ��Ƶ��(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//��ʼ��Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
}

/***************  PWM��ʼ������ *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//ģʽ,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWMռ�ձ�ʱ��, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//���ͨ��ѡ��,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//��ʼ��PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//ģʽ,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWMռ�ձ�ʱ��, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//���ͨ��ѡ��,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//��ʼ��PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Period   = PWM_Period; //2000							//����ʱ��,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//��������������, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//�����ʹ��, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//ʹ�ܼ�����, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//��ʼ��PWMͨ�üĴ���,  PWMA,PWMB

	PWM6_USE_P21();
	PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

void PWM_Left(int pwm)     //��0~100��ʾ
{	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //ע��PWM_Period��ʱΪ100���� 2000	
}

void PWM_Right(int pwm)     //��0~100��ʾ
{	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);	
}

void PWM_Run(int pwm1,int pwm2)  //����PWM
{
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

// 带方向的差速驱动：left/right 取值 -100~100，正=前进，负=反转。
// 实测左右接反，已对调：逻辑左轮=电机A(AIN/PWM5)，逻辑右轮=电机B(BIN/PWM6)。
void Motor_Drive(int left, int right)
{
	// 逻辑左轮 → 电机A 方向
	if (left >= 0) { AIN1=0; AIN2=1; }          // 前进
	else           { AIN1=1; AIN2=0; left=-left; }  // 反转
	// 逻辑右轮 → 电机B 方向
	if (right >= 0){ BIN1=1; BIN2=0; }          // 前进
	else           { BIN1=0; BIN2=1; right=-right; } // 反转

	// PWM_Run(pwm1=PWM6=电机B, pwm2=PWM5=电机A)，所以这里参数也要交换
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
// 把 (0, MIN_DRIVE) 之间的小速度抬到 MIN_DRIVE；真正要停的 0 保持 0。
int Floor(int v)
{
	if (v > 0 && v < MIN_DRIVE)  return MIN_DRIVE;
	return v;
}

// 计算线偏差。返回值范围约 -4(线在最左) ~ +4(线在最右)，0=居中。
// 传感器：读到 0 = 压黑线(在线上)，1 = 白底。物理排列 左→右：zuo1 zuo2 zhong you1 you2
int Track_GetError(void)
{
	int s_l1, s_l2, s_m, s_r1, s_r2;   // 压线=1，白底=0
	int cnt, sum;

	s_l1 = ON_LINE(zuo1);    // 最左   P0.4
	s_l2 = ON_LINE(zuo2);    // 次左   P0.3
	s_m  = ON_LINE(zhong);   // 中     P0.2
	s_r1 = ON_LINE(you1);    // 次右   P0.1
	s_r2 = ON_LINE(you2);    // 最右   P0.0

	cnt = s_l1 + s_l2 + s_m + s_r1 + s_r2;          // 压线的传感器个数
	sum = s_l1*(-4) + s_l2*(-2) + s_m*0 + s_r1*(+2) + s_r2*(+4);  // 位置加权和

	if (cnt == 0)        // 全白脱线 → 不更新记忆，靠上次方向找回（在主循环里处理）
		return last_error;

	if (cnt >= 5)        // 全黑(十字/路口) → 直行通过
	{
		last_error = 0;
		return 0;
	}

	last_error = (sum / cnt) * DIR_SIGN;            // 归一化偏差 + 左右纠正
	return last_error;
}

/******************** ������**************************/
void main(void)
{	int i;
	WTST = 0;		//���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();		//��չSFR(XFR)����ʹ�� 
	CKCON = 0;      //��߷���XRAM�ٶ�

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

		// 统计有几个传感器压在黑线上
		cnt = ON_LINE(zuo1) + ON_LINE(zuo2) + ON_LINE(zhong)
		    + ON_LINE(you1) + ON_LINE(you2);

		if (cnt == 0)
		{
			// === 全部脱线 ===
			// 朝最后偏离方向找线。用足够大的速度（带负载也能动），但不打死转圈。
			if (last_error > 0)             // 上次线偏右 → 向右找（左轮前进、右轮反转，原地右转找线）
				Motor_Drive(BASE_SPEED, -LOST_TURN);
			else if (last_error < 0)        // 上次线偏左 → 向左找
				Motor_Drive(-LOST_TURN, BASE_SPEED);
			else                            // 没有记忆（刚开机就脱线）→ 直行，别乱转
				Motor_Drive(BASE_SPEED, BASE_SPEED);
			continue;
		}

		// === 正常巡线：加权偏差 + 差速 ===
		error = Track_GetError();           // -4(线最左) ~ +4(线最右)
		turn  = error * KP;
		left  = BASE_SPEED + turn;          // error>0(偏右) → 左轮快、右轮慢/反转 → 右转追线
		right = BASE_SPEED - turn;
		// 限上限；允许为负(急转弯内轮反转)，但太小的正值抬到启动地板
		if (left  > MAX_SPEED) left  = MAX_SPEED;
		if (right > MAX_SPEED) right = MAX_SPEED;
		if (!TURN_HARD) { if(left<0) left=0; if(right<0) right=0; }  // 关掉反转时内轮只到0
		left  = Floor(left);                // 正小值抬到 MIN_DRIVE；负值/0 不动
		right = Floor(right);
		Motor_Drive(left, right);

		// 不加 delay：持续读传感器，避免转向时盲跑冲出
	}
}




