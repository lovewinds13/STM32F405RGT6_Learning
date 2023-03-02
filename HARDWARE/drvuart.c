//--------------------------------------------------------------------------------------------------------
//
//	模块名称 : uart收发操作
//	文件名称 : drvuart.c
//	版    本 : V1.0
//	说    明 : 
//				(1) drvuart.c 收发,轮询+中断
//				(2) 
//				(3) 
//				(4) 
//
//	修改记录 :
//		版本号     日期        作者     说明
//		V1.0    2021-05-25   霁风AI  正式发布
//		V1.1    
//		V1.2	
//		V1.3	
//
//	Copyright (C), 2020-2030, 微信公众号——TECHTIMES
//
//--------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVUART_C__
#define __PRJ_STM32F40X_DRVUART_C__

#include "drvuart.h"
#include "drvblehc05.h"
#include "drvesp8266.h"
#include "drvoled.h"
#include <stdio.h>
#include <string.h>

extern BLE_HC05_T g_t_ble;
extern WIFI_ESP_T g_t_wifi;

//需要配合Micro LIB 一起使用(√选微库)
int fputc(int ch, FILE *f)
{      
//	while((USART1->SR & 0X40) == 0);//循环发送,直到发送完毕   
//    USART1->DR = (uint8_t)ch;      
	
	 uart_send_byte(DEBUG_COMM_NO, (uint8_t)ch);
//	uart_send_byte(4, (uint8_t)ch);

//	uart_send_serial_bytes(1, (uint8_t *)&ch, 1);
	
	return ch;
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_gpio_init
//	功能说明: 串口GPIO初始化
//	形    参: 	uart_chl：串口编号
//	返 回 值: 无
//	日    期: 2020-03-25
//  备    注: 串口GPIO初始化
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_gpio_init(uint8_t uart_chl)
{
    GPIO_InitTypeDef gpio_config_init;

    if (uart_chl == 1)
    {
        RCC_AHB1PeriphClockCmd(RCC_PCLK_USART1_GPIO, ENABLE);	//使能UART1 GPIO时钟,AHB1总线
		
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(USART1_TXD_PORT, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9复用为USART1
		GPIO_PinAFConfig(USART1_RXD_PORT, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10复用为USART1

		gpio_config_init.GPIO_Pin 	= USART1_TXD_IO; //PA9
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART1_TXD_PORT, &gpio_config_init);//初始化GPIOA9

		gpio_config_init.GPIO_Pin 	= USART1_RXD_IO;//PA10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用功能必须启用
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART1_RXD_PORT, &gpio_config_init);//初始化GPIOA10
    }
   else if (uart_chl == 2)
    {
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//使能UART1 GPIO时钟,APB2总线
		
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(USART2_TXD_PORT, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART1
		GPIO_PinAFConfig(USART2_RXD_PORT, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART1

		gpio_config_init.GPIO_Pin 	= USART2_TXD_IO; //PA2
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART2_TXD_PORT, &gpio_config_init);//初始化GPIOA2

		gpio_config_init.GPIO_Pin 	= USART2_RXD_IO;//PA3
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用功能
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_DOWN;
		GPIO_Init(USART2_RXD_PORT, &gpio_config_init);//初始化GPIOA3
    }
    else if (uart_chl == 3)
    {
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//使能UART3 GPIO时钟,AHB1总线
		
		//串口1对应引脚复用映射
		GPIO_PinAFConfig(USART3_TXD_PORT, GPIO_PinSource10, GPIO_AF_USART3); //GPIOB10复用为USART3
		GPIO_PinAFConfig(USART3_RXD_PORT, GPIO_PinSource11, GPIO_AF_USART3); //GPIOB11复用为USART3

		gpio_config_init.GPIO_Pin 	= USART3_TXD_IO; //PB10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART3_TXD_PORT, &gpio_config_init);//初始化GPIOB10

		gpio_config_init.GPIO_Pin 	= USART3_RXD_IO;//PB11
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART3_RXD_PORT, &gpio_config_init);//初始化GPIOB11
    }
     else if (uart_chl == 4)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//使能UART4 GPIO时钟,AHB1总线
		
		//串口4对应引脚复用映射
		GPIO_PinAFConfig(UART4_TXD_PORT, GPIO_PinSource10, GPIO_AF_UART4); //GPIOC10复用为UART4
		GPIO_PinAFConfig(UART4_RXD_PORT, GPIO_PinSource11, GPIO_AF_UART4); //GPIOC11复用为UART4

		gpio_config_init.GPIO_Pin 	= UART4_TXD_IO; //PC10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART4_TXD_PORT, &gpio_config_init);//初始化GPIO PC10

		gpio_config_init.GPIO_Pin 	= UART4_RXD_IO;//PC11
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART4_RXD_PORT, &gpio_config_init);//初始化GPIO PC11
    }
     else if (uart_chl == 5)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//使能UART5 GPIO时钟,AHB1总线
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//使能UART5 GPIO时钟,AHB1总线

		//串口4对应引脚复用映射
		GPIO_PinAFConfig(UART5_TXD_PORT, GPIO_PinSource12, GPIO_AF_UART5); //GPIOC12复用为UART5
		GPIO_PinAFConfig(UART5_RXD_PORT, GPIO_PinSource2,  GPIO_AF_UART5); //GPIOD2复用为UART5

		gpio_config_init.GPIO_Pin 	= UART5_TXD_IO; //PC12
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART5_TXD_PORT, &gpio_config_init);//初始化GPIO PC12

		gpio_config_init.GPIO_Pin 	= UART5_RXD_IO;//PD2
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//浮空输入	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//复用推挽输出
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART5_RXD_PORT, &gpio_config_init);//初始化GPIO PD2
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_config
//	功能说明: 串口配置参数初始化
//	形    参: 	uart_chl：串口编号
//              uart_bound：波特率
//	返 回 值: 无
//	日    期: 2020-03-25
//  备    注: 串口编号从1-5(UART1-5),串口中断使能在Nvic设置里面配置
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_config(uint8_t uart_chl, uint32_t uart_bound)
{
    USART_InitTypeDef uart_config_init;

    if (uart_chl == 1)
    {
		RCC_APB2PeriphClockCmd(RCC_PCLK_USART1, ENABLE);	//APB2
		
		uart_config_init.USART_BaudRate              = uart_bound;	//串口波特率
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//字长为8位数据格式
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//一个停止位
		uart_config_init.USART_Parity                = USART_Parity_No;	//无奇偶校验位
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//无硬件数据流控制
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART1, &uart_config_init); //初始化串口1
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断(串口参数初始化之后才能开启中断)
		USART_Cmd(USART1, ENABLE); 		//使能串口1 
    }
    else if (uart_chl == 2)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_USART2, ENABLE);	//APB1
		
		uart_config_init.USART_BaudRate             = uart_bound;//串口波特率
		uart_config_init.USART_WordLength           = USART_WordLength_8b;//字长为8位数据格式
		uart_config_init.USART_StopBits             = USART_StopBits_1;//一个停止位
		uart_config_init.USART_Parity               = USART_Parity_No;//无奇偶校验位
		uart_config_init.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;//无硬件数据流控制
		uart_config_init.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART2, &uart_config_init); //初始化串口2
		// USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
		USART_Cmd(USART2, ENABLE); 		//使能串口2 
    }
    else if (uart_chl == 3)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_USART3, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//串口波特率
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//字长为8位数据格式
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//一个停止位
		uart_config_init.USART_Parity                = USART_Parity_No;	//无奇偶校验位
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//无硬件数据流控制
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(USART3, &uart_config_init); //初始化串口1
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断(串口参数初始化之后才能开启中断)
		USART_Cmd(USART3, ENABLE); 		//使能串口3 
    }
  else if (uart_chl == 4)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_UART4, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//串口波特率
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//字长为8位数据格式
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//一个停止位
		uart_config_init.USART_Parity                = USART_Parity_No;	//无奇偶校验位
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//无硬件数据流控制
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(UART4, &uart_config_init); //初始化串口4
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断(串口参数初始化之后才能开启中断)
		USART_Cmd(UART4, ENABLE); 		//使能串口4 
    }
      else if (uart_chl == 5)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_UART5, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//串口波特率
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//字长为8位数据格式
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//一个停止位
		uart_config_init.USART_Parity                = USART_Parity_No;	//无奇偶校验位
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//无硬件数据流控制
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

		USART_Init(UART5, &uart_config_init); //初始化串口5
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断(串口参数初始化之后才能开启中断)
		USART_Cmd(UART5, ENABLE); 		//使能串口5 
    }

}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_irq_set
//	功能说明: 串口中断初始化配置
//	形    参: 	uart_chl：串口编号
//	返 回 值: 无
//	日    期: 2020-03-27
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_irq_set(uint8_t uart_chl)
{
	NVIC_InitTypeDef uart_nvic_config;

	if (uart_chl == 1)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART1_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//抢占优先级3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//子优先级3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQ通道使能
		NVIC_Init(&uart_nvic_config);	//根据指定的参数初始化NVIC寄存器

//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//开启串口接受中断
    }
    else if (uart_chl == 2)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART2_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 3 ;//抢占优先级3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//子优先级3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQ通道使能
		NVIC_Init(&uart_nvic_config);	//根据指定的参数初始化NVIC寄存器

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
    }
    else if (uart_chl == 3)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART3_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//抢占优先级3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 2;		//子优先级3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQ通道使能
		NVIC_Init(&uart_nvic_config);	//根据指定的参数初始化NVIC寄存器

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
    }
    else if (uart_chl == 4)
    {
        uart_nvic_config.NVIC_IRQChannel 					= UART4_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//抢占优先级3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//子优先级3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQ通道使能
		NVIC_Init(&uart_nvic_config);	//根据指定的参数初始化NVIC寄存器

		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	//开启串口接受中断
    }
    else if (uart_chl == 5)
    {
        uart_nvic_config.NVIC_IRQChannel 					= UART5_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 3 ;//抢占优先级3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//子优先级3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQ通道使能
		NVIC_Init(&uart_nvic_config);	//根据指定的参数初始化NVIC寄存器

		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启串口接受中断
    }
    else 
    {
//        printf("error! \r\n");
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_init
//	功能说明: 串口G配置参数初始化
//	形    参: 	uart_chl：串口编号
//              uart_bound：波特率
//	返 回 值: 无
//	日    期: 2020-03-25
//  备    注: 统一调用底层接口(三部分：GPIO + UART + NVIC)
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
void uart_init(uint8_t uart_chl, uint32_t uart_bound)
{
    if (uart_chl == 1)
    {
        uart_gpio_init(uart_chl);
        uart_config(uart_chl, uart_bound);
		uart_irq_set(uart_chl);
    }
    else if (uart_chl == 2)
    {
        uart_gpio_init(uart_chl);
        uart_config(uart_chl, uart_bound);
		uart_irq_set(uart_chl);
    }
    else if (uart_chl == 3)
    {
        uart_gpio_init( uart_chl);
        uart_config(uart_chl, uart_bound);
		uart_irq_set(uart_chl);
    }
    else if (uart_chl == 4)
    {
        uart_gpio_init(uart_chl);
        uart_config(uart_chl, uart_bound);
		uart_irq_set(uart_chl);
    }
    else if (uart_chl == 5)
    {
        uart_gpio_init(uart_chl);
        uart_config(uart_chl, uart_bound);
		uart_irq_set(uart_chl);
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_send_byte
//	功能说明: 串口发送一字节数据
//	形    参: 	uart_chl：串口编号
//              send_byte：发送的数据
//	返 回 值: 无
//	日    期: 2020-03-27
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_send_byte(uint8_t uart_chl, uint8_t send_byte)
{
	if (uart_chl == 1)
    {
	#if 0
		while((USART1->SR & 0X40) == 0);//循环发送,直到发送完毕   
		USART1->DR = (uint8_t)send_byte; 
	#else
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//等待串口发送标志位置位

		USART_SendData(USART1, (uint8_t)send_byte);	//发送数据

	#endif
    }
    else if (uart_chl == 2)
    {
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	//等待串口发送标志位置位

		USART_SendData(USART2, (uint8_t)send_byte);	//发送数据
    }
    else if (uart_chl == 3)
    {
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	//等待串口发送标志位置位

		USART_SendData(USART3, (uint8_t)send_byte);	//发送数据
    }
    else if (uart_chl == 4)
    {
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	//等待串口发送标志位置位

		USART_SendData(UART4, (uint8_t)send_byte);	//发送数据
    }
    else if (uart_chl == 5)
    {
		while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);	//等待串口发送标志位置位

		USART_SendData(UART5, (uint8_t)send_byte);	//发送数据
    }
    else 
    {
//        printf("error! \r\n");
    }

	return 0;
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_send_serial_bytes
//	功能说明: 串口发送多字节数据
//	形    参: 	uart_chl：串口编号
//              pbdata：发送的数据地址
//				length：发送数据的长度
//	返 回 值: 无
//	日    期: 2020-03-28
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_send_serial_bytes(uint8_t uart_chl, uint8_t *pbdata, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		uart_send_byte(uart_chl, pbdata[i]);
	}
	
	// while (length--)
	// {
	// 	uart_send_byte(uart_chl, *pbdata++);
	// }

	return 0;
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_recv_byte
//	功能说明: 串口接收一字节数据
//	形    参: 	uart_chl：串口编号
//	返 回 值: recv_byte：接收的数据
//	日    期: 2020-03-28
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_recv_byte(uint8_t uart_chl)
{
	uint8_t recv_byte = 0;

	if (uart_chl == 1)
    {
	#if 0
		while((USART1->SR & 0X20) == 0);//循环发送,直到发送完毕   
		recv_byte = USART1->DR; 
	#else
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);	//等待串口接收标志位置位
		recv_byte = USART_ReceiveData(USART1);	//接收数据

	#endif
    }
    else if (uart_chl == 2)
    {
		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);	//等待串口接收标志位置位
		recv_byte = USART_ReceiveData(USART2);	//接收数据
    }
    else if (uart_chl == 3)
    {
		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);	//等待串口接收标志位置位
		recv_byte = USART_ReceiveData(USART3);	//接收数据
    }
    else if (uart_chl == 4)
    {
		while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);	//等待串口接收标志位置位
		recv_byte = USART_ReceiveData(UART4);	//接收数据
    }
    else if (uart_chl == 5)
    {
		while (USART_GetFlagStatus(UART5, USART_FLAG_RXNE) == RESET);	//等待串口接收标志位置位
		recv_byte = USART_ReceiveData(UART5);	//接收数据
    }
    else 
    {
//        printf("error! \r\n");
    }
	
	return recv_byte;
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_recv_serial_bytes
//	功能说明: 串口接收多字节数据
//	形    参: 	uart_chl：串口编号
//              pbdata：发送的数据地址
//				length：发送数据的长度
//	返 回 值: 无
//	日    期: 2020-03-28
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_recv_serial_bytes(uint8_t uart_chl, uint8_t *pbdata, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		*pbdata++ = uart_recv_byte(uart_chl);
	}
	
	return 0;
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: USART1_IRQHandler
//	功能说明: 串口中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-05-25
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_rx_buf[10];	
uint8_t uart_rx_len = 0;
void USART1_IRQHandler(void)
{
	uint8_t i = 0;
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//检查接收中断标志位
	{	
		if (uart_rx_len < 10)
			uart_rx_buf[uart_rx_len++] = USART_ReceiveData(USART1);
//			uart_rx_buf[uart_rx_len++] = uart_recv_byte(1);
		if (uart_rx_len >= 10)
		{
			uart_send_serial_bytes(1, uart_rx_buf, 10);
			uart_rx_len = 0;
			for (i=0; i<10; i++)
			{
				uart_rx_buf[i] = 0x00;
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	//清除串口接收中断标志
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: USART3_IRQHandler
//	功能说明: 串口3中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-09
//  备    注: 用于HC05模块的接收数据
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void)
{
	uint8_t i = 0;
	uint8_t ret_dat;
	
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)	//检查接收中断标志位
	{
//		TIM_SetCounter(TIM2, 0);	//计数器清零
//		TIM_Cmd(TIM2, ENABLE);	//还没有接收完成,继续监测
		
		ret_dat = USART_ReceiveData(USART3);
		
		if (g_t_ble.comm_mode != BLE_RX_OK)	//ble接收没有完成
		{
			if (g_t_ble.ble_rx_length < BLE_RX_LEN)	//ble接收数据小于限制长度
			{
				TIM_SetCounter(TIM2, 0);	//计数器清零
				if (g_t_ble.comm_mode == BLE_RX_MODE)
				{
					TIM_Cmd(TIM2, ENABLE);	//还没有接收完成,继续监测
				}
				g_t_ble.ble_rx_buff[g_t_ble.ble_rx_length++] = ret_dat;
				
	//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//清除串口接收中断标志
			}
			else 
			{
				g_t_ble.comm_mode = BLE_RX_OK;	//超出缓冲区长度,强制停止
			}
		}
		
//		g_t_ble.ble_length++;
//		oled_dis_str(2, 2, &g_t_ble.ble_buff[0]);
//		oled_dis_str(16, 2, &g_t_ble.ble_buff[1]);
//		oled_dis_num(2, 6, g_t_ble.ble_length, 3, 16, 0x80);
//		oled_dis_num(63, 6, g_t_ble.ble_buff[1], 3, 16, 0x80);

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//清除串口接收中断标志
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: UART4_IRQHandler
//	功能说明: 串口4中断服务函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-28
//  备    注: 用于WIFI模块的接收数据
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
void UART4_IRQHandler(void)
{
/*
	uint8_t i = 0;
	
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)	//检查接收中断标志位
	{	
		if (uart_rx_len < 10)
			uart_rx_buf[uart_rx_len++] = USART_ReceiveData(UART4);
//			uart_rx_buf[uart_rx_len++] = uart_recv_byte(1);
		if (uart_rx_len >= 10)
		{
			uart_send_serial_bytes(4, uart_rx_buf, 10);
			uart_rx_len = 0;
			for (i=0; i<10; i++)
			{
				uart_rx_buf[i] = 0x00;
			}
		}

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);	//清除串口接收中断标志
	}
*/
	uint8_t i = 0;
	uint8_t ret_dat;
	
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)	//检查接收中断标志位
	{
//		TIM_SetCounter(TIM4, 0);	//计数器清零
//		TIM_Cmd(TIM4, ENABLE);	//还没有接收完成,继续监测
		
		ret_dat = USART_ReceiveData(UART4);
		
		if (g_t_wifi.wifi_comm_mode != ESP_RX_OK)	//esp8266接收没有完成
		{
			if (g_t_wifi.wifi_rx_length < ESP_RX_LEN)	//esp8266接收数据小于限制长度
			{
				TIM_SetCounter(TIM4, 0);	//计数器清零
				if (g_t_wifi.wifi_comm_mode == ESP_RX_MODE)
				{
					TIM_Cmd(TIM4, ENABLE);	//还没有接收完成,继续监测
				}
				g_t_wifi.wifi_rx_buff[g_t_wifi.wifi_rx_length++] = ret_dat;
				
	//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//清除串口接收中断标志
			}
			else 
			{
				g_t_wifi.wifi_comm_mode = ESP_RX_OK;	//超出缓冲区长度,强制停止
			}
		}
		
//		g_t_wifi.wifi_rx_length++;
//		oled_dis_str(2, 2, &g_t_wifi.wifi_rx_buff[0]);
//		oled_dis_str(16, 2, &g_t_wifi.wifi_rx_buff[1]);
//		oled_dis_num(2, 6, g_t_wifi.wifi_rx_length, 3, 16, 0x80);
//		oled_dis_num(63, 6, g_t_wifi.wifi_rx_buff[1], 3, 16, 0x80);

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);	//清除串口接收中断标志
	}
}

void UART5_IRQHandler(void)
{
	uint8_t i = 0;
	
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)	//检查接收中断标志位
	{	
		if (uart_rx_len < 10)
			uart_rx_buf[uart_rx_len++] = USART_ReceiveData(UART5);
//			uart_rx_buf[uart_rx_len++] = uart_recv_byte(1);
		if (uart_rx_len >= 10)
		{
			uart_send_serial_bytes(5, uart_rx_buf, 10);
			uart_rx_len = 0;
			for (i=0; i<10; i++)
			{
				uart_rx_buf[i] = 0x00;
			}
		}

		USART_ClearITPendingBit(UART5, USART_IT_RXNE);	//清除串口接收中断标志
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	函 数 名: uart_test
//	功能说明: 串口收发测试函数
//	形    参: 无
//	返 回 值: 无
//	日    期: 2020-03-28
//  备    注: 无
//	作    者: by 霁风AI
//-------------------------------------------------------------------------------------------------------------------------
void uart_test(void)
{
	uint8_t tmp_buff[6] = {0};
	uint8_t err_flag[6] = "error";
	
	tmp_buff[0] = uart_rx_buf[0];

//	uart_recv_serial_bytes(1, tmp_buff, 1);

	if (tmp_buff[0] != 0x00)
	{
		uart_send_serial_bytes(1, tmp_buff, sizeof(tmp_buff));
	}
	else
	{
//		uart_send_serial_bytes(1, err_flag, 6);
	}
}
#endif /* __PRJ_STM32F40X_DRVUART_C__ */
