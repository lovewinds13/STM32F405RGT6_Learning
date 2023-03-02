//-----------------------------------------------------------------------------------------------------------
//
//	模块名称 : BSP模块(For STM32F405RGT6)
//	文件名称 : bsp.c
//	版    本 : V1.0
//	说    明 : 这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
//			   bsp = Borad surport packet 板级支持包
//	修改记录 :
//		版本号  日期         作者       说明
//		V1.0    2021-05-25  霁风AI   正式发布
//
//	Copyright (C), 2021-2030, 微信公众号 TECHTIMES
//
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//	STM32F405RGT6开发板LED口线分配：
//		LED0     : PA1          (低电平点亮，高电平熄灭)
//		LED1     : PA2          (低电平点亮，高电平熄灭)
//------------------------------------------------------------------------------------------------------------

#include "drvled.h"
#include "delay.h"

//#define DRIVER_LED_REG	//寄存器方式选择控制开关

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void Bsp_LedInit(void)
{
#ifdef DRIVER_LED_REG
	RCC->APB2ENR	|= 1<<2;	//GPIOA 时钟使能
	RCC->APB2ENR	|= 1<<5;	//GPIOD 时钟使能
	
	GPIOA->CRH		&= 0xfffffff0;	
	GPIOA->CRH		|= 0x00000003;	//推挽输出,IO口速度为50MHz
	GPIOA->ODR		|= 1<<8;	//PA8 输出高电平
	
	GPIOD->CRL		&= 0xfffff0ff;
	GPIOD->CRL		|= 0x00000300;	//推挽输出,IO口速度为50MHz
	GPIOD->ODR		|= 1<<2;	//PD2 输出高电平
#else
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(LED_GPIO_CLK_ALLENABLE, ENABLE);	 //使能PA端口时钟

	gp_init.GPIO_Pin 	= LED0_CON_IO;				 //LED0-->PA.1 端口配置
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//普通输出模式
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //推挽输出
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		 //IO口速度为50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//上拉
	GPIO_Init(LED0_GPIO_PORT, &gp_init);					 	

	gp_init.GPIO_Pin 	= LED1_CON_IO;	    		 //LED1-->PA.2 端口配置, 推挽输出
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//普通输出模式
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //推挽输出
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//推挽输出 ，IO口速度为50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//上拉
	GPIO_Init(LED1_GPIO_PORT, &gp_init);	  				
	
	GPIO_SetBits(LED0_GPIO_PORT, LED0_CON_IO);
	GPIO_SetBits(LED1_GPIO_PORT, LED1_CON_IO); 		
#endif	
}

//-------------------------------------------------------------------------------------------------------
//	函 数 名: bsp_LedOn
//	功能说明: 点亮指定的LED指示灯,灌电流点亮
//	形    参:  _no : 指示灯序号，范围 0 - 1
//	返 回 值: 无
//-------------------------------------------------------------------------------------------------------
void Bsp_LedOn(uint8_t _no)
{
#ifdef DRIVER_LED_REG
	if (_no == 0)
	{
		 GPIOA->ODR |= 0x0100;//1<<8;
	}
	else if (_no == 1)
	{
		 GPIOD->ODR |= 1<<2;
	}
#else	
	if (_no == 0)
	{
		 GPIO_ResetBits(LED0_GPIO_PORT, LED0_CON_IO);
	}
	else if (_no == 1)
	{
		 GPIO_ResetBits(LED1_GPIO_PORT, LED1_CON_IO);
	}
#endif
}

//-------------------------------------------------------------------------------------------------------
//	函 数 名: Bsp_LedOff
//	功能说明: 关闭指定的LED指示灯
//	形    参:  _no : 指示灯序号，范围 0 - 1
//	返 回 值: 无
//-------------------------------------------------------------------------------------------------------
void Bsp_LedOff(uint8_t _no)
{	
#ifdef DRIVER_LED_REG
	if (_no == 0)
	{
		 GPIOA->ODR &= 0xfeff;//0<<8;
	}
	else if (_no == 1)
	{
//		 GPIOD->ODR &= 0<<2;
		GPIOD->ODR	&= ~(1<<2);
	}
#else
	if (_no == 0)
	{
		 GPIO_SetBits(LED0_GPIO_PORT, LED0_CON_IO);
	}
	else if (_no == 1)
	{
		 GPIO_SetBits(LED1_GPIO_PORT, LED1_CON_IO);
	}
#endif
}

//-------------------------------------------------------------------------------------------------------
//	函 数 名: bsp_LedToggle
//	功能说明: 翻转指定的LED指示灯。
//	形    参:  _no : 指示灯序号，范围 0 - 1
//	返 回 值: 无
//-------------------------------------------------------------------------------------------------------
void Bsp_LedToggle(uint8_t _no)
{
	uint8_t flag = _no % 2;
	
	if (flag == 0)
	{
		LED0_GPIO_PORT->ODR ^= LED0_CON_IO;
	}
	else if (flag == 1)
	{
		LED1_GPIO_PORT->ODR ^= LED1_CON_IO;
	}
}

//-------------------------------------------------------------------------------------------------------
//	函 数 名: Bsp_LedTest
//	功能说明: LED灯驱动测试。
//	形    参:  _uiTime : 延时时间
//	返 回 值: 无
//-------------------------------------------------------------------------------------------------------
void Bsp_LedTest(uint16_t _uiTime)
{
	Bsp_LedToggle(0);
	delay_ms(_uiTime);
	Bsp_LedToggle(1);
	delay_ms(_uiTime);
}


//***************************** 微信公众号 TECHTIMES (END OF FILE) *********************************/
