#ifndef __PRJ_STM32F40X_DRVBLEHC05_C__
#define __PRJ_STM32F40X_DRVBLEHC05_C__

//---------------------------------------------------------------------------------------------------------------------------------------------
//平    台:				STM32F40X
//文    件:    		 	drvblehc05.c
//作    者:       		霁风AI
//库版 本:   			Vxxx
//文件版本:   			V1.0.0
//日   期:      		2021年06月04日
//说   明:      	 	蓝牙(HC05)驱动实现
//----------------------------------------------------------------------------------------------------------------------------------------------

#include "drvuart.h"
#include "drvblehc05.h"
#include "drvoled.h"
#include "drvtimer.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"

BLE_HC05_T g_t_ble;

__align(8) uint8_t ble_rx_buff[BLE_RX_LEN];	//开辟一个400字节的存储空间
__align(8) uint8_t ble_tx_buff[BLE_TX_LEN];

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_gpio_init
//	功能说明: BLE使能引脚配置
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-05
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t ble_gpio_init(void)
{
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(RCC_PCLK_BLE_GPIO, ENABLE);	 //使能PB端口时钟
	
	gp_init.GPIO_Pin 	= BLE_EN_IO;	    		 //PB12 端口配置, 推挽输出
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//普通输出模式
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //推挽输出
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//推挽输出 ，IO口速度为50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//上拉
	GPIO_Init(PORT_BLE_IN, &gp_init);	  				
	
	GPIO_SetBits(PORT_BLE_IN, BLE_EN_IO);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_sent_at
//	功能说明: BLE 发送AT指令
//	形    参: at_cmd：AT 指令
//	返 回 值: 无
//	日    期: 2021-06-05
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void ble_sent_cmd(const char *at_cmd)
{
	uart_send_serial_bytes(BLE_COMM_NO, (uint8_t *)at_cmd, strlen(at_cmd));
	uart_send_serial_bytes(BLE_COMM_NO, "\r\n", 2);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_init
//	功能说明: BLE 初始化操作
//	形    参: 无
//	返 回 值: 0
//	日    期: 2021-06-19
//  备    注: 通过定时器来计算串口数据接收是否完成
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ble_init(void)
{
	g_t_ble.comm_mode = BLE_DE_MODE;	//初始化为接收模式,用于检测HC05模块
	ble_rx_buff[0] = 'A';
	ble_rx_buff[1] = 'T';
	g_t_ble.ble_rx_length = 0;
	g_t_ble.ble_rx_buff = ble_rx_buff;
	
#if EN_IO_MODE	//1：需要通过IO来进入AT模式; 0：上电进入AT模式
	ble_gpio_init();
	uart_init(BLE_COMM_NO, 9600);
//	timer_init(2);	//用定时器来检测串口数据的接收情况
#else 
	uart_init(BLE_COMM_NO, 9600);
	timer_init(2);	//用定时器来检测串口数据的接收情况
	TIM_Cmd(TIM2, DISABLE);
#endif	
	
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_detect
//	功能说明: BLE 模块检测,判断初始化是否成功
//	形    参: 无
//	返 回 值: 0
//	日    期: 2021-06-19
//  备    注: 测试AT指令
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ble_detect(void)
{
	uint8_t count = 30;
	uint16_t i = 0;
	uint8_t ret_val = 0xFF;
	
	g_t_ble.comm_mode = BLE_RX_MODE;	//接收模式
	
	while (count--)
	{
		#if EN_IO_MODE
			BLE_EN_1();
			ble_sent_cmd("AT");
			delay_ms(10);
			BLE_EN_0();
		#else
			ble_sent_cmd("AT");
			delay_ms(2);
		#endif
		
//		if (g_t_ble.ble_rx_length)
//		{
//			printf("ble_length is %d \r\n", g_t_ble.ble_rx_length);
//			while (i <= g_t_ble.ble_rx_length)
//			{
//				printf("ble recv: %C", g_t_ble.ble_rx_buff[i++]);
//			}
//			printf("\r\n");
//		}
//		else 
//		{
//			printf("no recv -- ble_length is %d \r\n", g_t_ble.ble_rx_length);
//		}
		
		if ((g_t_ble.ble_rx_buff[0]) == 'O' && (g_t_ble.ble_rx_buff[1] == 'K'))
		{
			printf("ble_length is %d \r\n", g_t_ble.ble_rx_length);
//			for (i = 0; i < 4; i++)	
				printf("ble recv: %c \r\n", g_t_ble.ble_rx_buff[0]);
			printf("ble recv: %c \r\n", g_t_ble.ble_rx_buff[1]);
			oled_dis_str(2, 2, "ble ok!");
			
			printf("\r\n ble rx flag: %d \r\n\r\n", g_t_ble.comm_mode);
			
			ret_val = 0;
			break;
		}
		
 		if (count == 0)
		{
			ret_val = 1;
//			g_t_ble.ble_buff[0] = 'A';
			oled_dis_str(2, 6, "ble err!");
//			oled_dis_str(2, 2, &g_t_ble.ble_rx_buff[0]);
//			oled_dis_str(16, 2, &g_t_ble.ble_rx_buff[1]);
//			oled_dis_str(2, 4, "count == 0");
		}
	}
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_cmd_test
//	功能说明: BLE 命令测试
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-19
//  备    注: 测试常用AT指令
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void ble_cmd_test(void)
{
	uint16_t i;
	uint8_t count = 30;
	
	g_t_ble.ble_rx_length = 0;
	for (i = 0; i < BLE_RX_LEN; i++)
	{
		g_t_ble.ble_rx_buff[i] = 0x00;
	}
	
	g_t_ble.comm_mode = BLE_RX_MODE;	//接收模式
	
	char cmd_buf[] = "AT+NAME?";
	//AT+VERSION?
	//AT+ROLE?
	//AT+NAME?
	//AT+ADDR?
	while (count--)
	{
		ble_sent_cmd(cmd_buf);
		delay_ms(2);
		
		if (g_t_ble.comm_mode == BLE_RX_OK)
		{
			printf("\r\ncmd ret length is %d \r\n", g_t_ble.ble_rx_length);
			for (i = 0; i < g_t_ble.ble_rx_length; i++)	
			{
				printf("\r\nble recv cmd: %c \r\n", g_t_ble.ble_rx_buff[i]);
			}
			printf("\r\nble recv cmd: %s \r\n", g_t_ble.ble_rx_buff);
			
			break;
		}
		if (count == 0)
		{
			printf("\r\nble recv falied!\r\n");
		}
	}
	
	ble_reset_ref();
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: ble_reset_ref
//	功能说明: BLE 相关操作恢复初始化
//	形    参: 无
//	返 回 值: 无
//	日    期: 2021-06-19
//  备    注: 参数恢复默认状态
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void ble_reset_ref(void)
{
	uint16_t i;
	
	if (g_t_ble.comm_mode == BLE_RX_OK)
	{
		g_t_ble.comm_mode = BLE_RX_MODE;	//接收模式
		
		g_t_ble.ble_rx_length = 0;
		for (i = 0; i < BLE_RX_LEN; i++)
		{
			g_t_ble.ble_rx_buff[i] = 0x00;
		}
	}
}

#endif /* __PRJ_STM32F40X_DRVBLEHC05_C__ */
