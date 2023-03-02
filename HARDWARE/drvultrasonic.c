//---------------------------------------------------------------------------------------------------------------------------------------------
//平    台:				STM32F10X
//文    件:    		 	drultrasonic.c
//作    者:       		霁风AI
//库版 本:   			Vxxx
//文件版本:   			V1.0.0
//日   期:      		2021年04月11日
//说   明:      	 	超声波模块 HC-SR04 驱动实现
//----------------------------------------------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVULTRASONIC_C__
#define __PRJ_STM32F40X_DRVULTRASONIC_C__

#include "delay.h"
#include "drvtimer.h"
#include "drvuart.h"
#include "drvultrasonic.h"
#include "stdio.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: hcsr04_gpio_init
//	功能说明: 超声波模块GPIO配置
//	形    参: 无
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-04-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void hcsr04_gpio_init(void)
{
	GPIO_InitTypeDef gpio_initx;
	  
	RCC_AHB1PeriphClockCmd(RCC_PCLK_ULT_GPIO, ENABLE);	//使能GPIOB时钟
	
	//PA0 作为输出触发引脚                         
	gpio_initx.GPIO_Pin		= ULT_TRIG_PIN;	//TRIG引脚
	gpio_initx.GPIO_Speed 	= GPIO_Speed_50MHz;		 //IO口速度为50MHz
	gpio_initx.GPIO_Mode 	= GPIO_Mode_OUT;	//输出
	gpio_initx.GPIO_OType 	= GPIO_OType_PP;	//推挽输出
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_UP; 
	GPIO_Init(PORT_ULT_GPIO, &gpio_initx);	
	
		//PA1 作为输入检测引脚                         
	gpio_initx.GPIO_Pin		= ULT_ECHO_PIN;	//ECHO引脚
	gpio_initx.GPIO_Speed 	= GPIO_Speed_50MHz;
	gpio_initx.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_DOWN;		//下拉输入引脚
	GPIO_Init(PORT_ULT_GPIO, &gpio_initx);
	
	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);

#if 1	//启动一个定时器来计时间
	TIM_TimeBaseInitTypeDef timer_init_config;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	timer_init_config.TIM_Period 				= 65535;	//自动重装载寄存器数值
	timer_init_config.TIM_Prescaler 			= 84;	//时钟频率除数预分频数值(计数一次1us)
	timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//时钟分割
	timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//向上计数方式
	//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//高级定时器使用
	TIM_TimeBaseInit(TIM4, &timer_init_config);
	
//	TIM_SetCounter(TIM3, 0);	//定时器的计数初值设置为0
	TIM_Cmd(TIM4, ENABLE);	//定时器使能
#endif
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: hcsr04_start
//	功能说明: 超声波模块启动配置
//	形    参: 无
//	返 回 值: 无
//  备    注: 发送一个大于10us的高电平
//	日    期: 2021-04-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void hcsr04_start(void)
{
//	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
//	delay_us(2);
	GPIO_SetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
	delay_us(12);	//高电平时间大于10us
	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: hcsr04_read_data
//	功能说明: 超声波模块读取测量数据
//	形    参: 无
//	返 回 值: tim_cnt:声波往返时间
//  备    注: 定时器来得到声波往返时间
//	日    期: 2021-04-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint16_t hcsr04_read_data(void)
{
	uint16_t tim_cnt = 0;
	uint16_t time_start, time_end;
	
	hcsr04_start();
	
#if 1	//定时器计时
	TIM_SetCounter(TIM4, 0);
	while (!ULT_ECHO_READ);	//低电平等待
	
	time_start = TIM_GetCounter(TIM4);
	while (ULT_ECHO_READ)	//高电平读取数据
	{
		;
	}
	time_end = TIM_GetCounter(TIM4);
	
	tim_cnt = time_end - time_start;
	
#else	
	while (!ULT_ECHO_READ);	//低电平等待
	
	while (ULT_ECHO_READ)	//高电平读取数据
	{
		tim_cnt++;	//运行时间??
		delay_us(1);
	}
	tim_cnt = tim_cnt * 2;
#endif	
	return tim_cnt;	//时间us
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: hcsr04_measure
//	功能说明: 超声波模块计算测量距离
//	形    参: 无
//	返 回 值: distance:测量结果
//  备    注: 多次测量取平均值
//	日    期: 2021-04-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
float hcsr04_measure(void)
{
	uint8_t i;
	float distance = 0.0;
	uint16_t temp, sum_val = 0;
	
	for (i = 0; i < 5; i++)
	{
		temp = hcsr04_read_data();
//		printf("No : %d ; Value is %d \r\n", i, temp);
		sum_val += temp;
		delay_ms(60);
	}
	
	temp = sum_val / 5;	//5次平均
//	printf("average value is %d \r\n", temp);
//	distance = ((temp / 1000000) * 340 * 100) / 2;	//以 cm 为单位
	distance = (float)temp / 58;
	
	return distance;
}


#endif	//__PRJ_STM32F40X_DRVULTRASONIC_C__
