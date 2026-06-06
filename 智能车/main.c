/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码，请在程序中注明使用了STC的资料及程序            */
/*---------------------------------------------------------------------*/

#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
#include  "iic.h"
#include  "font.h"
#include  "oled.h"




/*************  项目说明  **************

智能车 5 路灰度循迹 + L298P 双电机。
巡线控制放在 Timer2 中断里，每 1ms 执行一次 Track_Control()。
PWM5/PWM6 调左右电机转速，AIN/BIN 控制方向。
时钟 24MHz（在 config.h 修改）。

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
sbit TRIG = P0^6;
sbit ECHO = P1^4;
sbit KEY1 = P2^2;
sbit KEY2 = P2^3;
sbit KEY3 = P2^4;
sbit KEY4 = P1^3;
sbit KEY5 = P1^7;

volatile unsigned int time_us = 0;
bit measuring = 0;

PWMx_Duty PWMB_Duty;
u16 PWM_Period = 2000; // 2000
bit PWM5_Flag;
bit PWM6_Flag;

/* ===== 巡线参数（5路灰度，压黑线为1）— PD比例控制 =====
   调参口诀（一次只动一个）：
   - 画龙/抖：先加 KD（治画龙主力），再降 BASE_PWM。
   - 转弯不够/冲出：加 KP。
   - 越修越偏/反向追线：DIR 改 -1。
*/
#define LINE_ON             1       /* 传感器有效电平：压黑线=1，白底=0 */
#define CTRL_DIV_1MS        10      /* Timer2为100us中断，10次执行一次控制≈1ms */

#define BASE_PWM            62      /* 直道基础速度。画龙先降这个 */
#define KP                  6       /* 比例：偏多少修多少。转弯不够就加大，修正太猛就调小 */
#define KD                  12      /* 微分：抑制过冲/阻尼。画龙就加大（治画龙主力旋钮）*/
#define DIR                 (+1)    /* 修正方向：越修越偏就改 -1 */
#define ERR_FILTER          6       /* 偏差平滑系数(0~10)：越大越平滑(治顿挫)但越滞后。修正顿挫就加大 */

/* ---- 直角弯检测 ---- */
#define SHARP_OUT_PWM       90      /* 直角弯外轮速度 */
#define SHARP_IN_PWM        42      /* 直角弯内轮速度(低=转得急) */
#define SHARP_HOLD_TICKS    90      /* 直角弯急转锁存时间(ms)：横线一闪而过，靠锁存撑过拐点。转不够就加大 */

#define MAX_PWM             92      /* PWM最大限幅 */
#define MIN_RUN_PWM         42      /* 不停车时的最低轮速，必须大于电机起转阈值 */
#define BASE_NODE           70      /* 路口/全黑等走直的速度 */
#define LEFT_TRIM           0       /* 左轮微调：直线往一边偏就调，范围-10~10 */
#define RIGHT_TRIM          0       /* 右轮微调：同上 */
#define LOST_TURN_PWM       85      /* 脱线找线外轮速度 */
#define LOST_TURN_INNER     30      /* 脱线找线内轮速度，越低转向越急 */

#define LOST_STRAIGHT_TICKS 18      /* 短暂全白先保持直行多久(ms)，过虚线/十字不慌 */
#define START_PWM           85      /* 起步速度：压在起点方块上时全速冲出 */

#define MASK_ZUO2           0x10    /* 最左传感器 */
#define MASK_ZUO1           0x08    /* 次左传感器 */
#define MASK_ZHONG          0x04    /* 中间传感器 */
#define MASK_YOU1           0x02    /* 次右传感器 */
#define MASK_YOU2           0x01    /* 最右传感器 */

int last_error = 0;                 /* 上一周期(平滑后)偏差，用于算 D 项 */
int error_filt = 0;                 /* 平滑后的偏差(低通滤波，治顿挫) */
u16 lost_ticks = 0;
u8 start_square_mode = 1;
u8 ctrl_divider = 0;
int last_left_pwm = BASE_PWM;
int last_right_pwm = BASE_PWM;


/*************  本地函数声明  **************/

int LimitPwm(int pwm);
void PWM_Run(int pwm1,int pwm2);
void Motor_RunSafe(int left_pwm, int right_pwm);
void Motor_HoldLast(void);
u8 ReadTrackSensors(void);
u8 CountBits(u8 mask);
int Track_GetError(u8 mask);
void Track_NormalControl(u8 mask, u8 count);
void Track_Control(void);

/*************  外部函数和变量声明 *****************/



/************************ IO初始化 ****************************/
void	GPIO_config(void)
{


}

/************************ 定时器0初始化 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					// 结构体
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	// 16位自动重装模式
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			// 1T时钟源
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				// 不输出时钟
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		// 中断频率 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					// 预分频 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				// 启动定时器
	Timer_Inilize(Timer0,&TIM_InitStructure);				// 初始化 Timer0
	NVIC_Timer0_Init(ENABLE,Priority_0);					// 中断使能，优先级0
}

void Timer1_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 1T时钟（快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出时钟
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 100000UL)); // 10us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 启动定时器

    Timer_Inilize(Timer1, &TIM_InitStructure);                          // 初始化定时器1（超声波计时用）
    NVIC_Timer1_Init(ENABLE, Priority_0);                               // 中断使能，优先级0
}

void Timer2_config(void)
{
    TIM_InitTypeDef TIM_InitStructure;

    TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;              // 16位自动重装
    TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;                     // 1T时钟（快）
    TIM_InitStructure.TIM_ClkOut    = DISABLE;                          // 不输出时钟
		TIM_InitStructure.TIM_Value 		= (u16)(65536UL - (MAIN_Fosc / 10000UL)); // 100us定时
    TIM_InitStructure.TIM_PS        = 0;                                // 不预分频
    TIM_InitStructure.TIM_Run       = ENABLE;                           // 启动定时器

    Timer_Inilize(Timer2, &TIM_InitStructure);                          // 初始化定时器2（巡线控制节拍）
    NVIC_Timer2_Init(ENABLE, Priority_0);                               // 中断使能，优先级0
}

/***************  超声波计时中断 *****************/
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		// 每10us进一次，测距时累加
{
	if (measuring) time_us++;
}

unsigned int Ultrasonic_GetDistance(void) 
{
    unsigned int Distance_mm = 0;
    TRIG = 0;
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 1;
    _nop_(); _nop_(); _nop_(); _nop_();    // 约10us高电平触发
    _nop_(); _nop_(); _nop_(); _nop_();
    _nop_(); _nop_(); _nop_(); _nop_();
    TRIG = 0;

    // 等待 ECHO 拉高（回波开始）
    while (!ECHO);

    // 开始计时
    time_us = 0;
    measuring = 1;

    // 等待 ECHO 拉低（回波结束）
    while (ECHO);

    measuring = 0;
		if(time_us/100<38)									// 小于38ms才有效，否则超时返回0
		{
			Distance_mm=(time_us*346)/100;					// 算距离：声速346m/s，time单位10us，故乘346除100
		}
    return Distance_mm;
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;		// PWM模式1
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	// 占空比 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;				// 输出通道 PWM5正
	PWM_Configuration(PWM5, &PWMx_InitStructure);			// 初始化 PWM5（电机A调速）

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;		// PWM模式1
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	// 占空比 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;				// 输出通道 PWM6正
	PWM_Configuration(PWM6, &PWMx_InitStructure);			// 初始化 PWM6（电机B调速）

	PWMx_InitStructure.PWM_Period   = PWM_Period;			// 周期 2000
	PWMx_InitStructure.PWM_DeadTime = 0;					// 死区时间 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;			// 主输出使能
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;			// 计数器使能
	PWM_Configuration(PWMB, &PWMx_InitStructure);			// 初始化 PWMB 通用寄存器


		PWM6_USE_P21();
		PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

int LimitPwm(int pwm)
{
	if(pwm < 0) return 0;
	if(pwm > MAX_PWM) return MAX_PWM;
	return pwm;
}

void Motor_RunSafe(int left_pwm, int right_pwm)
{
	/* 只有明确给(0,0)时才真正停车；巡线/转弯时保证每个轮都有最低运转PWM */
	if(left_pwm == 0 && right_pwm == 0)
	{
		last_left_pwm = 0;
		last_right_pwm = 0;
		PWM_Run(0, 0);
		return;
	}

	left_pwm += LEFT_TRIM;
	right_pwm += RIGHT_TRIM;
	left_pwm = LimitPwm(left_pwm);
	right_pwm = LimitPwm(right_pwm);

	if(left_pwm < MIN_RUN_PWM) left_pwm = MIN_RUN_PWM;
	if(right_pwm < MIN_RUN_PWM) right_pwm = MIN_RUN_PWM;

	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
	PWM_Run(left_pwm, right_pwm);
}

void Motor_HoldLast(void)
{
	/* 短暂丢线时保持上一次的左右速度，让动作延续（过虚线/十字不慌） */
	if(last_left_pwm == 0 && last_right_pwm == 0)
	{
		Motor_RunSafe(BASE_PWM, BASE_PWM);
	}
	else
	{
		PWM_Run(last_left_pwm, last_right_pwm);
	}
}
void PWM_Left(int pwm)     // 入参0~100
{

	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  // PWM_Period此时为100倍即2000

}

void PWM_Right(int pwm)    // 入参0~100
{

	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);

}


void PWM_Run(int pwm1,int pwm2)  // 设置左右PWM
{
	pwm1 = LimitPwm(pwm1);
	pwm2 = LimitPwm(pwm2);
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

void test01(void)
{
		Motor_RunSafe(100,80);
		delay_ms(1000);
		Motor_RunSafe(80,100);
		delay_ms(1000);
		Motor_RunSafe(MIN_RUN_PWM,100);
		delay_ms(1000);
		Motor_RunSafe(100,MIN_RUN_PWM);
		delay_ms(1000);
}

u8 ReadTrackSensors(void)
{
	/* 读取5路循迹：最左/次左/中/次右/最右，组合成bit掩码 */
	u8 mask;
	mask = 0;
	if(zuo2 == LINE_ON)  mask |= MASK_ZUO2;
	if(zuo1 == LINE_ON)  mask |= MASK_ZUO1;
	if(zhong == LINE_ON) mask |= MASK_ZHONG;
	if(you1 == LINE_ON)  mask |= MASK_YOU1;
	if(you2 == LINE_ON)  mask |= MASK_YOU2;
	return mask;
}

u8 CountBits(u8 mask)
{
	u8 count;
	count = 0;
	if(mask & MASK_ZUO2)  count++;
	if(mask & MASK_ZUO1)  count++;
	if(mask & MASK_ZHONG) count++;
	if(mask & MASK_YOU1)  count++;
	if(mask & MASK_YOU2)  count++;
	return count;
}

/* 位置加权算偏差：5路按位置加权(×10放大避免整数除法丢精度)，再除以压线数。
   返回约 -40(线最左) ~ +40(线最右)，0=居中。连续量，供PD用。 */
int Track_GetError(u8 mask)
{
	int sum = 0;
	u8 cnt = 0;

	if(mask & MASK_ZUO2) { sum += -40; cnt++; }   /* 最左 */
	if(mask & MASK_ZUO1) { sum += -20; cnt++; }   /* 次左 */
	if(mask & MASK_ZHONG){ sum +=   0; cnt++; }   /* 中   */
	if(mask & MASK_YOU1) { sum +=  20; cnt++; }   /* 次右 */
	if(mask & MASK_YOU2) { sum +=  40; cnt++; }   /* 最右 */

	if(cnt == 0) return 0;
	return (sum / cnt) * DIR;
}

void Track_NormalControl(u8 mask, u8 count)
{
	int error, derr, turn, left, right;

	/* 00000全白脱线：先保持一小段直行，超时后朝上次偏离方向急找线 */
	if(count == 0)
	{
		if(lost_ticks < 60000) lost_ticks++;
		if(lost_ticks <= LOST_STRAIGHT_TICKS)
		{
			Motor_HoldLast();
		}
		else if(last_error >= 0)
		{
			Motor_RunSafe(LOST_TURN_PWM, LOST_TURN_INNER);   /* 朝右急找线 */
		}
		else
		{
			Motor_RunSafe(LOST_TURN_INNER, LOST_TURN_PWM);   /* 朝左急找线 */
		}
		return;
	}
	lost_ticks = 0;

	/* 直角弯急转锁存中：横线一闪而过，靠锁存强制急转撑过拐点，期间不看PD。
	   提前结束条件：中间传感器重新压上新线(已对正)。 */
	if(sharp_hold > 0)
	{
		sharp_hold--;
		if(mask & MASK_ZHONG)              /* 已对正新线，提前结束 */
		{
			sharp_hold = 0;
			last_error = 0;
		}
		else if(sharp_dir > 0)
			Motor_RunSafe(SHARP_OUT_PWM, SHARP_IN_PWM);   /* 右转 */
		else
			Motor_RunSafe(SHARP_IN_PWM, SHARP_OUT_PWM);   /* 左转 */
		return;
	}

	/* 路口/十字/方块：4路及以上同时压线，按直行处理，不修正 */
	if(count >= 4)
	{
		last_error = 0;
		Motor_RunSafe(BASE_NODE, BASE_NODE);
		return;
	}

	/* 直角弯检测：最外侧压线 + 对侧白 → 触发急转锁存。
	   横线扫过最外传感器那一瞬就抓住，靠锁存撑过去。 */
	if((mask & MASK_YOU2) && !(mask & MASK_ZUO1))      /* 右直角 */
	{
		sharp_dir = +1;
		sharp_hold = SHARP_HOLD_TICKS;
		Motor_RunSafe(SHARP_OUT_PWM, SHARP_IN_PWM);
		return;
	}
	if((mask & MASK_ZUO1) && !(mask & MASK_YOU2))      /* 左直角 */
	{
		sharp_dir = -1;
		sharp_hold = SHARP_HOLD_TICKS;
		Motor_RunSafe(SHARP_IN_PWM, SHARP_OUT_PWM);
		return;
	}

	/* ===== 正常巡线：PD 比例控制 ===== */
	error = Track_GetError(mask);          /* 原始偏差 -40~+40（离散，5路只有几档）*/

	/* 低通滤波：让偏差在档位之间平滑过渡，消除"台阶跳变→猛修"的顿挫感。
	   error_filt = 旧×F/10 + 新×(10-F)/10 */
	error_filt = (error_filt * ERR_FILTER + error * (10 - ERR_FILTER)) / 10;

	derr  = error_filt - last_error;       /* 偏差变化率（D项核心）*/
	last_error = error_filt;

	/* turn = P + D；÷10 抵消 error 放大的10倍，让 KP/KD 取小整数 */
	turn = (KP * error_filt + KD * derr) / 10;

	/* error>0(线偏右) → 要右转 → 左轮快、右轮慢 */
	left  = BASE_PWM + turn;
	right = BASE_PWM - turn;

	Motor_RunSafe(left, right);            /* 内含限幅+MIN_RUN保底，纯差速不停轮 */
}

void Track_Control(void)
{
	u8 mask;
	u8 count;

	mask = ReadTrackSensors();
	count = CountBits(mask);

	/* 起步阶段：上电时压在起点方块上(5路全黑)，直行冲出方块 */
	if(start_square_mode)
	{
		if(count == 5)
		{
			Motor_RunSafe(START_PWM, START_PWM);
			return;
		}
		start_square_mode = 0;
	}

	Track_NormalControl(mask, count);
}

void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		// 巡线控制中断
{
	/* Timer2每100us进一次，分频后约1ms执行一次控制，保证响应及时 */
	ctrl_divider++;
	if(ctrl_divider >= CTRL_DIV_1MS)
	{
		ctrl_divider = 0;
		Track_Control();
	}
}
/******************** 主函数 **************************/
void main(void)
{
//	unsigned char i;
	WTST = 0;		// 设置取指令时序，0=CPU执行指令最快
	EAXSFR();		// 扩展SFR(XFR)访问使能
	CKCON = 0;      // 提高访问XRAM速度

	P2M1=0x00;
	P2M0=0xFF;
	P2M0 &= ~(1 << 2);  // M0=0		p2.2
	P2M0 &= ~(1 << 3);  // M0=0		p2.3
	P2M0 &= ~(1 << 4);  // M0=0		p2.4	
	P0M1=0x00;
	P0M0=0x00;
	P1M1=0x00;
	P1M0=0x00;
	P4M1=0x00;
	P4M0=0xFF;	
	P1M1 |= (1 << 4);   // M1=1
	P1M0 &= ~(1 << 4);  // M0=0	
	
	Timer_config();
	Timer1_config();
	Timer2_config();
	PWM_config();
	oled_Init();
	
	EA = 1;
	
	AIN1=0;
	AIN2=1;
	BIN1=1;
	BIN2=0;

	TRIG = 0;
	ECHO = 0;
	time_us = 0;
	while (1)
	{
//			unsigned int d = Ultrasonic_GetDistance();
//			oled_clear();
//			oled_ShowNum(56,2,d,5,16);
//			oled_ShowNum(10,2,1,2,16);		
//			delay_ms(300);	

//		if(KEY4 == 0)
//		{
//			LED1=0;
//			delay_ms(1000);
//			LED1=1;
//			delay_ms(1000);
//		}
//		if(KEY5 == 0)
//		{
//			LED2=0;
//			delay_ms(1000);
//			LED2=1;
//			delay_ms(1000);
//		}
//		if(KEY3 == 0)
//		{
//			LED3=0;
//			delay_ms(1000);
//			LED3=1;
//			delay_ms(1000);
//		}
//		for(i=10;i>0;i--)
//		{
//			oled_clear();
//			oled_ShowNum(10,2,i,2,16);		
//			delay_ms(1000);
//		}



//		if(T0_1ms)
//		{
//			
//			PWMB_Duty.PWM5_Duty = 2047;
//			PWMB_Duty.PWM6_Duty = 1;
//			T0_1ms = 0;
			
//			if(!PWM5_Flag)
//			{
//				PWMB_Duty.PWM5_Duty++;
//				if(PWMB_Duty.PWM5_Duty >= 2047) PWM5_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM5_Duty--;
//				if(PWMB_Duty.PWM5_Duty <= 0) PWM5_Flag = 0;
//			}
//			if(!PWM6_Flag)
//			{
//				PWMB_Duty.PWM6_Duty++;
//				if(PWMB_Duty.PWM6_Duty >= 2047) PWM6_Flag = 1;
//			}
//			else
//			{
//				PWMB_Duty.PWM6_Duty--;
//				if(PWMB_Duty.PWM6_Duty <= 0) PWM6_Flag = 0;
//			}
			
			
		}
	}




