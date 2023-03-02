#ifndef __PRJ_STM32F40X_DRVTIMER_C__
#define __PRJ_STM32F40X_DRVTIMER_C__

#include "drvtimer.h"
#include "drvled.h"
#include "drvoled.h"
#include "drvblehc05.h"
#include "drvesp8266.h"

//#define DRIVER_TIMER_REG	//补充寄存器方式

extern BLE_HC05_T g_t_ble;
extern WIFI_ESP_T g_t_wifi;

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: timer_config
//	功能说明: 定时器参数配置
//	形    参: 	timer_no：定时器编号
//				timer_arr：自动重装值
//				timer_psc：时钟分频系数
//	返 回 值: 无
//	日    期: 2020-04-14
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void timer_config(uint8_t timer_no, uint16_t timer_arr, uint16_t timer_psc)
{
#ifdef DRIVER_TIMER_REG	//寄存器方式
	if (timer_no == 3)
	{
		RCC->APB1ENR	|= 1 << 1;	//定时器3时钟使能(APB1)
		TIM3->ARR		 = timer_arr;	//自动重装载计数器赋值
		TIM3->PSC		 = timer_psc;	//预分频器赋值
		TIM3->DIER		|= 1 << 0;	//中断更新使能
		TIM3->CR1	 	|= 0x01;	//使能定时器3
	}
#else	
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
	if (timer_no == 3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//自动重装载寄存器数值
		timer_init_config.TIM_Prescaler 			= timer_psc;	//时钟频率除数预分频数值
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//时钟分割
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//向上计数方式
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//高级定时器使用
		TIM_TimeBaseInit(TIM3, &timer_init_config);
	}
	if (timer_no == 4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//自动重装载寄存器数值
		timer_init_config.TIM_Prescaler 			= timer_psc;	//时钟频率除数预分频数值
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//时钟分割
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//向上计数方式
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//高级定时器使用
		TIM_TimeBaseInit(TIM4, &timer_init_config);
	}
#endif	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: timer_nvic_config
//	功能说明: 定时器NVIC参数配置
//	形    参: 	timer_no：定时器编号
//	返 回 值: 无
//	日    期: 2020-04-14
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
#ifndef DRIVER_TIMER_REG
static void timer_nvic_config(uint8_t timer_no)
{
	NVIC_InitTypeDef nvic_init_config;
	
	if (timer_no == 2)
	{
		//中断优先级 NVIC 设置
		nvic_init_config.NVIC_IRQChannel 						= TIM2_IRQn; //TIM2 中断
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 1; //先占优先级 1 级
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 1; //从优先级 1 级
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ 通道被使能
		NVIC_Init(&nvic_init_config); //初始化 NVIC 寄存器
		
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//允许更新中断
	}

	if (timer_no == 3)
	{
		//中断优先级 NVIC 设置
		nvic_init_config.NVIC_IRQChannel 						= TIM3_IRQn; //TIM3 中断
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 3; //先占优先级 2 级
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 3; //从优先级 3 级
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ 通道被使能
		NVIC_Init(&nvic_init_config); //初始化 NVIC 寄存器
		
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//允许更新中断
	}
	
	if (timer_no == 4)
	{
		//中断优先级 NVIC 设置
		nvic_init_config.NVIC_IRQChannel 						= TIM4_IRQn; //TIM4 中断
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 0; //先占优先级 1 级
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 1; //从优先级 1 级
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ 通道被使能
		NVIC_Init(&nvic_init_config); //初始化 NVIC 寄存器
		
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//允许更新中断
	}
}
#endif

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: timer_init
//	功能说明: 定时器初始化配置
//	形    参: 	timer_no：定时器编号
//	返 回 值: 无
//	日    期: 2020-04-14
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------

void timer_init(uint8_t timer_no)
{
#ifdef DRIVER_TIMER_REG
	
#else
	if (timer_no == 2)
	{
		timer_config(timer_no, 100-1, 8400-1);//10ms中断一次
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM2, ENABLE);//开启定时器
	}
	
	if (timer_no == 3)
	{
		timer_config(timer_no, 4999, 8399);//计数满载5000 = 500ms;计数频率:84M/8400=10KHz
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM3, ENABLE);//开启定时器
	}
	
	if (timer_no == 4)
	{
		timer_config(timer_no, 1000-1, 8400-1);//100ms中断一次
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM4, ENABLE);//开启定时器TIM4
	}
#endif	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: TIM2_IRQHandler
//	功能说明: 定时器2中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-09
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		g_t_ble.comm_mode = BLE_RX_OK;	//10ms没有再收到数据表示完成
//		g_t_ble.comm_mode = 99;
//		Bsp_LedToggle(0);
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	TIM_Cmd(TIM2, DISABLE);	//停止计数
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: TIM3_IRQHandler
//	功能说明: 定时器3中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-05-27
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		Bsp_LedToggle(1);	
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: TIM4_IRQHandler
//	功能说明: 定时器4中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-28
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		g_t_wifi.wifi_comm_mode = ESP_RX_OK;	//100ms没有再收到数据表示完成
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	
	TIM_Cmd(TIM4, DISABLE);	//停止计数
}


#endif	//__PRJ_STM32F40X_DRVTIMER_C__
