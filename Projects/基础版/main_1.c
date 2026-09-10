#include	"config.h"
#include	"STC32G_PWM.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_NVIC.h"
#include	"STC32G_Timer.h"
#include	"STC32G_Delay.h"
/*************	功能说明	**************
高级PWM定时器 PWM5,PWM6,PWM7,PWM8 每个通道都可独立实现PWM输出.
4个通道PWM根据需要设置对应输出口，可通过示波器观察输出的信号.
PWM周期和占空比可以自定义设置，最高可达65535.
下载时, 选择时钟 24MHZ (用户可在"config.h"修改频率).
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
bit PWM5_Flag;
bit PWM6_Flag;

/*************	本地函数声明	**************/

/*************  外部函数和变量声明 *****************/

/************************ IO口配置 ****************************/
void	GPIO_config(void)
{
	
}

/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000UL));		//中断频率, 1000次/秒
	TIM_InitStructure.TIM_PS        = 0;					//8位预分频器(n+1), 0~255
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer0_Init(ENABLE,Priority_0);		//中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3
}

/***************  PWM初始化函数 *****************/
void	PWM_config(void)
{
	PWMx_InitDefine		PWMx_InitStructure;

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM5_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO5P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM5, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Mode    =	CCMRn_PWM_MODE1;	//模式,		CCMRn_FREEZE,CCMRn_MATCH_VALID,CCMRn_MATCH_INVALID,CCMRn_ROLLOVER,CCMRn_FORCE_INVALID,CCMRn_FORCE_VALID,CCMRn_PWM_MODE1,CCMRn_PWM_MODE2
	PWMx_InitStructure.PWM_Duty    = PWMB_Duty.PWM6_Duty;	//PWM占空比时间, 0~Period
	PWMx_InitStructure.PWM_EnoSelect   = ENO6P;					//输出通道选择,	ENO1P,ENO1N,ENO2P,ENO2N,ENO3P,ENO3N,ENO4P,ENO4N / ENO5P,ENO6P,ENO7P,ENO8P
	PWM_Configuration(PWM6, &PWMx_InitStructure);				//初始化PWM,  PWMA,PWMB

	PWMx_InitStructure.PWM_Period   = PWM_Period; //2000							//周期时间,   0~65535
	PWMx_InitStructure.PWM_DeadTime = 0;								//死区发生器设置, 0~255
	PWMx_InitStructure.PWM_MainOutEnable= ENABLE;				//主输出使能, ENABLE,DISABLE
	PWMx_InitStructure.PWM_CEN_Enable   = ENABLE;				//使能计数器, ENABLE,DISABLE
	PWM_Configuration(PWMB, &PWMx_InitStructure);				//初始化PWM通用寄存器,  PWMA,PWMB

	PWM6_USE_P21();
	PWM5_USE_P20();
	NVIC_PWM_Init(PWMB,DISABLE,Priority_0);
}

void PWM_Left(int pwm)     //用0~100表示
{	
	PWMB_Duty.PWM6_Duty = pwm*(PWM_Period/100);  //注意PWM_Period此时为100倍数 2000	
}

void PWM_Right(int pwm)     //用0~100表示
{	
	PWMB_Duty.PWM5_Duty = pwm*(PWM_Period/100);	
}

void PWM_Run(int pwm1,int pwm2)  //左右PWM
{
	PWM_Left(pwm1);
	PWM_Right(pwm2);
	UpdatePwm(PWMB, &PWMB_Duty);
}

void test01(void)
{
		PWM_Run(100,80);
		delay_ms(1000);
		PWM_Run(80,100);
		delay_ms(1000);
		PWM_Run(0,100);
		delay_ms(1000);
		PWM_Run(100,0);
		delay_ms(1000);
}
/******************** 主函数**************************/
void main(void)
{
	int i;
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度

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
	while (1)
	{
		if(zuo2==0 && zuo1==0 && zhong==1 && you1==1 && you2==1)   //三个右
		{
			PWM_Run(80,0);
			delay_ms(130);
		}
		else if(zuo2==1 && zuo1==1 && zhong==1 && you1==0 && you2==0) //三个左
		{
			PWM_Run(0,80);
			delay_ms(130);
		}
		else if (zuo2==0 && zuo1==0 && zhong==0 && you1==1 && you2==1)   //两个右
		{
			PWM_Run(90,0);
			delay_ms(130);
		}		
		else if (zuo2==0 && zuo1==0 && zhong==1 && you1==0 && you2==1)   //两个右
		{
			PWM_Run(85,0);
			delay_ms(130);
		}		
		else if (zuo2==0 && zuo1==0 && zhong==1 && you1==1 && you2==0)   //两个右
		{
			PWM_Run(90,50);
			delay_ms(130);
		}
		else if (zuo2==1 && zuo1==1 && zhong==0 && you1==0 && you2==0)   //两个左
		{
			PWM_Run(0,90);
			delay_ms(130);
		}
		else if (zuo2==1 && zuo1==0 && zhong==1 && you1==0 && you2==0)   //两个左
		{
			PWM_Run(0,85);
			delay_ms(130);
		}
		else if (zuo2==0 && zuo1==1 && zhong==1 && you1==0 && you2==0)   //两个左
		{
			PWM_Run(50,90);
			delay_ms(130);
		}		
		else if (zuo2==0 && zuo1==0 && zhong==0 && you1==0 && you2==1)    //一个右
		{
			PWM_Run(90,0);
		}
		else if (zuo2==0 && zuo1==0 && zhong==0 && you1==1 && you2==0)    //一个右
		{
			PWM_Run(85,50);
		}
		else if (zuo2==1 && zuo1==0 && zhong==0 && you1==0 && you2==0)    //一个左
		{
			PWM_Run(0,90);
		}
		else if (zuo2==0 && zuo1==1 && zhong==0 && you1==0 && you2==0)    //一个左
		{
			PWM_Run(50,85);
		}
		else if(zuo1==1 && you1==1  && zhong==1)                      //三个中           
		{
		  PWM_Run(95,95);
			delay_ms(50);
		}
		else if (zuo2==0 && zuo1==1 && (zhong==0 || zhong==1) && you1==1 && you2==0) //mid
		{
			PWM_Run(95,95);
		}		
		else if (zuo2==1 && zuo1==1 && zhong==0 && you1==1 && you2==1) //左右俩
		{
			PWM_Run(80,80);
		}		
		else if(zuo2==1 && zuo1==0 && zhong==0 && you1==0 && you2==1)  //左右一
		{
			PWM_Run(80,80);
		}
		else if(zuo2==0 && zuo1==1 && zhong==0 && you1==1 && you2==0)  //左右一
		{
			PWM_Run(80,80);
		}	
		else if (zuo2==0 && zuo1==0 && zhong==1 && you1==0 && you2==0) //mid
		{
			PWM_Run(90,90);
		}
		else if(zuo2==0 && zuo1==0 && zhong==0 && you1==0 && you2==0)  //全白 旋转
		{
			PWM_Run(90,0);
		}
		
		if(zuo2==1 && zuo1==1 && zhong==1 && you1==1 && you2==1) 
		{
				AIN1=0;
				AIN2=0;
				BIN1=0;
				BIN2=0;
				PWM_Run(0,0);
		}
		else
		{
				AIN1=0;
				AIN2=1;
				BIN1=1;
				BIN2=0;
		}
	
			
	}
}




