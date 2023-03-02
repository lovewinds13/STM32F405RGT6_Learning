#ifndef __PRJ_STM32F40X_DRVBLEHC05_C__
#define __PRJ_STM32F40X_DRVBLEHC05_C__

//---------------------------------------------------------------------------------------------------------------------------------------------
//ƽ    ̨:				STM32F40X
//��    ��:    		 	drvblehc05.c
//��    ��:       		����AI
//��� ��:   			Vxxx
//�ļ��汾:   			V1.0.0
//��   ��:      		2021��06��04��
//˵   ��:      	 	����(HC05)����ʵ��
//----------------------------------------------------------------------------------------------------------------------------------------------

#include "drvuart.h"
#include "drvblehc05.h"
#include "drvoled.h"
#include "drvtimer.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"

BLE_HC05_T g_t_ble;

__align(8) uint8_t ble_rx_buff[BLE_RX_LEN];	//����һ��400�ֽڵĴ洢�ռ�
__align(8) uint8_t ble_tx_buff[BLE_TX_LEN];

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: ble_gpio_init
//	����˵��: BLEʹ����������
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-05
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t ble_gpio_init(void)
{
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(RCC_PCLK_BLE_GPIO, ENABLE);	 //ʹ��PB�˿�ʱ��
	
	gp_init.GPIO_Pin 	= BLE_EN_IO;	    		 //PB12 �˿�����, �������
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//��ͨ���ģʽ
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //�������
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//������� ��IO���ٶ�Ϊ50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//����
	GPIO_Init(PORT_BLE_IN, &gp_init);	  				
	
	GPIO_SetBits(PORT_BLE_IN, BLE_EN_IO);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: ble_sent_at
//	����˵��: BLE ����ATָ��
//	��    ��: at_cmd��AT ָ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-05
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void ble_sent_cmd(const char *at_cmd)
{
	uart_send_serial_bytes(BLE_COMM_NO, (uint8_t *)at_cmd, strlen(at_cmd));
	uart_send_serial_bytes(BLE_COMM_NO, "\r\n", 2);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: ble_init
//	����˵��: BLE ��ʼ������
//	��    ��: ��
//	�� �� ֵ: 0
//	��    ��: 2021-06-19
//  ��    ע: ͨ����ʱ�������㴮�����ݽ����Ƿ����
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ble_init(void)
{
	g_t_ble.comm_mode = BLE_DE_MODE;	//��ʼ��Ϊ����ģʽ,���ڼ��HC05ģ��
	ble_rx_buff[0] = 'A';
	ble_rx_buff[1] = 'T';
	g_t_ble.ble_rx_length = 0;
	g_t_ble.ble_rx_buff = ble_rx_buff;
	
#if EN_IO_MODE	//1����Ҫͨ��IO������ATģʽ; 0���ϵ����ATģʽ
	ble_gpio_init();
	uart_init(BLE_COMM_NO, 9600);
//	timer_init(2);	//�ö�ʱ������⴮�����ݵĽ������
#else 
	uart_init(BLE_COMM_NO, 9600);
	timer_init(2);	//�ö�ʱ������⴮�����ݵĽ������
	TIM_Cmd(TIM2, DISABLE);
#endif	
	
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: ble_detect
//	����˵��: BLE ģ����,�жϳ�ʼ���Ƿ�ɹ�
//	��    ��: ��
//	�� �� ֵ: 0
//	��    ��: 2021-06-19
//  ��    ע: ����ATָ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ble_detect(void)
{
	uint8_t count = 30;
	uint16_t i = 0;
	uint8_t ret_val = 0xFF;
	
	g_t_ble.comm_mode = BLE_RX_MODE;	//����ģʽ
	
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
//	�� �� ��: ble_cmd_test
//	����˵��: BLE �������
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-19
//  ��    ע: ���Գ���ATָ��
//	��    ��: by ����AI
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
	
	g_t_ble.comm_mode = BLE_RX_MODE;	//����ģʽ
	
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
//	�� �� ��: ble_reset_ref
//	����˵��: BLE ��ز����ָ���ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-19
//  ��    ע: �����ָ�Ĭ��״̬
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void ble_reset_ref(void)
{
	uint16_t i;
	
	if (g_t_ble.comm_mode == BLE_RX_OK)
	{
		g_t_ble.comm_mode = BLE_RX_MODE;	//����ģʽ
		
		g_t_ble.ble_rx_length = 0;
		for (i = 0; i < BLE_RX_LEN; i++)
		{
			g_t_ble.ble_rx_buff[i] = 0x00;
		}
	}
}

#endif /* __PRJ_STM32F40X_DRVBLEHC05_C__ */
