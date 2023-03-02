//---------------------------------------------------------------------------------------------------------------------------------------------
//平    台:				STM32F40X
//文    件:    		 	drultrasonic.c
//作    者:       		霁风AI
//库版 本:   			Vxxx
//文件版本:   			V1.0.0
//日   期:      		2021年05月29日
//说   明:      	 	超声波模块 HC-SR04 驱动实现
//----------------------------------------------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVPWM_C__
#define __PRJ_STM32F40X_DRVPWM_C__


#include "drvpwm.h"
//#include "delay.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: pwm_gpio_init
//	功能说明: PWM输出GPIO配置
//	形    参: 无
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-05-30
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_gpio_init(void)
{
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(PWM1_GPIO_CLK_ALLENABLE, ENABLE);	 //使能PA端口时钟
	GPIO_PinAFConfig(PWM1_GPIO_PORT, GPIO_PinSource1, GPIO_AF_TIM2); //GPIOA1复用为TIM2_CH2
	
	gp_init.GPIO_Pin 	= PWM1_CON_IO;			//LPA.1 端口配置
	gp_init.GPIO_Mode 	= GPIO_Mode_AF;			//复用输出模式
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		//推挽输出
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//IO口速度为50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//上拉
	GPIO_Init(PWM1_GPIO_PORT, &gp_init);					 	 
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: pwm_timer_init
//	功能说明: PWM 定时器参数配置
//	形    参: 	timer_no：定时器编号
//				timer_arr：自动重装值
//				timer_psc：时钟分频系数
//	返 回 值: 无
//	日    期: 2020-05-30
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_timer_init(uint8_t timer_no, uint16_t timer_arr, uint16_t timer_psc)
{
	TIM_TimeBaseInitTypeDef timer_init_config;
	
	if (timer_no == 2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//自动重装载寄存器数值
		timer_init_config.TIM_Prescaler 			= timer_psc;	//时钟频率除数预分频数值
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//时钟分割
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//向上计数方式
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//高级定时器使用
		TIM_TimeBaseInit(TIM2, &timer_init_config);
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: pwm_configure
//	功能说明: PWM 参数配置
//	形    参: 无
//	返 回 值: 无
//	日    期: 2020-05-30
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_configure(void)
{
	TIM_OCInitTypeDef pwm_init_config;
	
	pwm_init_config.TIM_OCMode		= TIM_OCMode_PWM2;	//PWM通道2输出
	pwm_init_config.TIM_OutputState	= TIM_OutputState_Enable;	//PWM输出使能
	pwm_init_config.TIM_OCPolarity	= TIM_OCPolarity_Low;	//输出极性为低
	TIM_OC2Init(TIM2, &pwm_init_config);
	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);	//预加载使能
	TIM_ARRPreloadConfig(TIM2, ENABLE);	//ARPE使能
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: pwm_init
//	功能说明: PWM 初始化参数配置
//	形    参: pwm_no：输出序列
//	返 回 值: 无
//	日    期: 2020-05-30
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_init(uint8_t pwm_no)
{
	if (pwm_no == 1)
	{
		pwm_gpio_init();
		pwm_timer_init(2, 500-1, 84-1);
		pwm_configure();
		
		TIM_Cmd(TIM2, ENABLE);
	}
}

#endif	//__PRJ_STM32F40X_DRVPWM_C__
