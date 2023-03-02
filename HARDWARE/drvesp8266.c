#ifndef __PRJ_STM32F40X_DRVESP8266_C__
#define __PRJ_STM32F40X_DRVESP8266_C__

//---------------------------------------------------------------------------------------------------------------------------------------------
//ƽ    ̨:				STM32F40X
//��    ��:    		 	drvesp8266.c
//��    ��:       		����AI
//��� ��:   			Vxxx
//�ļ��汾:   			V1.0.0
//��   ��:      		2021��06��24��
//˵   ��:      	 	Wifi(ESP8266)����ʵ��
//----------------------------------------------------------------------------------------------------------------------------------------------

#include "drvuart.h"
#include "drvesp8266.h"
#include "drvoled.h"
#include "drvtimer.h"
#include "delay.h"
#include "string.h"
#include "stdio.h"

__align(8) uint8_t wifi_rx_buf[ESP_RX_LEN];	
__align(8) uint8_t wifi_tx_buf[ESP_TX_LEN];

WIFI_ESP_T g_t_wifi;

const uint8_t* wifi_sta_ssid = "HUAWEI Mate 20";	//�ȵ�(·����)����
const uint8_t* wifi_sta_pd = "xyz131420";	//����
uint8_t* wifi_sta_mode = "TCP";	//TCP(͸��)
const uint8_t* wifi_sta_ip = "192.168.43.182";	//Tcp sever ip
const uint8_t* wifi_sta_port = "50003";	//port

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_init
//	����˵��: ESP8266��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-26
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void esp8266_init(void)
{
//	uint8_t tmp_buf[2] = {0x55, 0xAA};
	g_t_wifi.wifi_comm_mode = ESP_DE_MODE;
	g_t_wifi.wifi_rx_buff = wifi_rx_buf;
	g_t_wifi.wifi_rx_length = 0;
	
	uart_init(ESP8266_COMM_NO, 115200);
	
	timer_init(4);	//��ʱ�����WIFI���ڽ�������
	TIM_Cmd(TIM4, DISABLE);
	
//	printf("esp8266 uart ok!\r\n");
	
//	uart_send_serial_bytes(ESP8266_COMM_NO, tmp_buf, 2);	//���Դ��ڷ�������
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_send_at_cmd
//	����˵��: ������ESP8266����ATָ��
//	��    ��: at_cmd��ATָ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-28
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void esp8266_send_at_cmd(const char *at_cmd)
{
	uart_send_serial_bytes(ESP8266_COMM_NO, (uint8_t *)at_cmd, strlen(at_cmd));
	uart_send_serial_bytes(ESP8266_COMM_NO, "\r\n", 2);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_check_at_ack
//	����˵��: ESP8266 ģ����ATָ���Ƿ�ɹ�
//	��    ��: ptr_cmd��ATָ���Ӧ�ķ��ؼ��("OK")
//	�� �� ֵ: ptr_val����鵽���ݶεĵ�ֵַ
//	��    ��: 2021-06-29
//  ��    ע: ���� strstr ����,����Ӧ���ַ���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t* esp8266_check_at_ack(uint8_t *ptr_cmd)
{
	char *ptr_val;
	
	if (g_t_wifi.wifi_comm_mode == ESP_RX_OK)
	{
		ptr_val = strstr((const char*)g_t_wifi.wifi_rx_buff, (const char*)ptr_cmd);
	}
	
	return (uint8_t*)ptr_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_send_data
//	����˵��: ESP8266 ��������
//	��    ��: 	dat_buf�����͵�����
//				ans_ack����Ӧ���ݶ�
//				wait_time����ʱ�ȴ�ʱ��
//	�� �� ֵ: 0���ɹ�;1��ʧ��
//	��    ��: 2021-06-29
//  ��    ע: ���� strstr ����,����Ӧ���ַ���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t esp8266_send_data(uint8_t *dat_buf, uint8_t *ans_ack, uint16_t wait_time)
{
	uint8_t *ack_val = 0;
	uint8_t ret_val = 0;
	int i;
	
//	g_t_wifi.wifi_comm_mode = ESP_RX_MODE;
	
	esp8266_send_at_cmd((char*)dat_buf);
	
	if (wait_time&&ans_ack)
	{
		while (wait_time--)
		{
			delay_ms(10);
			
			if (g_t_wifi.wifi_comm_mode == ESP_RX_OK)
			{
				ack_val = esp8266_check_at_ack(ans_ack);
				if(ack_val)
				{
//					oled_dis_str(2, 4, "wifi ok!");
//					
//					printf("wifi_length is %d \r\n", g_t_wifi.wifi_rx_length);
//			
//					for (i = 0; i < g_t_wifi.wifi_rx_length; i++)	
					{
//						printf("***wifi recv: 0x%02x \r\n", g_t_wifi.wifi_rx_buff[i]);
//						printf("***wifi recv: %c \r\n", g_t_wifi.wifi_rx_buff[i]);
//						if (g_t_wifi.wifi_rx_buff[i] == 'O' || g_t_wifi.wifi_rx_buff[i] == 'K')
//						{
//							printf("\r\n cmd is %s --- ack is %c \r\n", dat_buf, g_t_wifi.wifi_rx_buff[i]);
//						}
//						else 
//						{
//							printf("\r\n cmd is %s --- \r\n", "ack err");
//						}
					}
					break;
				}
				else 
				{
					oled_dis_str(2, 4, "wifi err!");
				}
			}
		}
		if (wait_time == 0)
		{
			ret_val = 1;
		}
	}
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_uart_trans
//	����˵��: ESP8266 ͸��ģʽ����
//	��    ��: 	trans_mode��ģʽѡ��(TCP or UDP)
//	�� �� ֵ: 0���ɹ�;1��ʧ��
//	��    ��: 2021-06-29
//  ��    ע: ���� strstr ����,����Ӧ���ַ���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t udp_mode[] = "UDP";
uint8_t udp_ip[] = "192.168.43.220";
uint8_t udp_port[] = "50004";

uint8_t esp8266_uart_trans(uint8_t trans_mode)
{
	uint16_t i;
	uint8_t at_buf1[] = "AT+CWMODE=1";	//����ΪSTAģʽ
	uint8_t at_buf2[] = "AT+RST";	//��λ
	uint8_t at_buf3[50];
//	uint8_t at_buf3_0[] = {0x41,0x54,0x2B,0x43,0x57,0x4A,0x41,0x50,0x3D,0x22,0x48,0x55,0x41,0x57,0x45,0x49,0x20,0x4D,0x61,0x74,0x65,0x20,0x32,0x30,0x22,0x2C,0x22,0x78,0x79,0x7A,0x31,0x33,0x31,0x34,0x32,0x30,0x22,};
	uint8_t at_buf4[] = "AT+CIFSR";	//��ѯģ���IP
	uint8_t at_buf5[50];	//���ӵ�sever
	uint8_t at_buf6[] = "AT+CIPMODE=1";	//����͸��ģʽ
	uint8_t at_buf7[] = "AT+CIPSEND";	//��ʹ͸��
	
	//ָ�����鶨���ַ���
	uint8_t* at_ins_buf[] = {
		"AT+CWMODE=1",
		"AT+RST",
		"AT+CIFSR",
		"AT+CIPMODE=1",
		"AT+CIPSEND"
	};
	
	//AT+CWJAP="HUAWEI Mate 20","wwt139229";
	sprintf((char*)at_buf3,"AT+CWJAP=\"%s\",\"%s\"", wifi_sta_ssid, wifi_sta_pd);
	
	if (WIFI_STA_TCP_C == trans_mode)	//TCP Client ģʽ����
	{
		oled_dis_str(2, 0, "tcp client");
		
		//AT+CIPSTART="TCP","192.168.43.182",50003
		sprintf((char*)at_buf5,"AT+CIPSTART=\"%s\",\"%s\",%s",wifi_sta_mode, wifi_sta_ip, wifi_sta_port);
		
		//1.ģʽ����Ϊsta
		esp8266_reset_ref();
		esp8266_send_data(at_buf1, "OK", 20);
		delay_ms(1000);
		//2.ģ�鸴λ
		esp8266_reset_ref();
		esp8266_send_data(at_buf2, "OK", 50);
		delay_ms(4000);
		//3.����·����(�������ӵ��ֻ��ȵ�)
		esp8266_reset_ref();
		while (esp8266_send_data(at_buf3, "WIFI GOT IP", 800))
		{
			oled_dis_str(2, 2, "wifi net");
			delay_ms(1000);
			oled_dis_clear();
		}
		delay_ms(1000);
		//	esp8266_reset_ref();
		//	esp8266_send_data(at_buf4, "OK", 300);
		//	delay_ms(1000);
		//4.���ӵ�sever
		esp8266_reset_ref();
		while (esp8266_send_data(at_buf5, "OK", 200))
		{
			oled_dis_str(2, 2, "wifi sever");
			delay_ms(1000);
			oled_dis_clear();
		}
		delay_ms(1000);
		//5.����͸��ģʽ
		esp8266_reset_ref();
		esp8266_send_data(at_buf6, "OK", 200);
		delay_ms(1000);
		//6.��ʼ͸��
		esp8266_reset_ref();
		esp8266_send_data(at_buf7, "OK", 50);
		esp8266_reset_ref();
	}
	
	else if (WIFI_STA_UDP == trans_mode)
	{			
		oled_dis_str(2, 0, "udp");
		
		//AT+CIPSTART="UDP","192.168.43.182",50003
		sprintf((char*)at_buf5,"AT+CIPSTART=\"%s\",\"%s\",%s",udp_mode, udp_ip, udp_port);
		
	//1.ģʽ����Ϊsta
		esp8266_reset_ref();
		esp8266_send_data(at_buf1, "OK", 20);
		delay_ms(1000);
		//2.ģ�鸴λ
		esp8266_reset_ref();
		esp8266_send_data(at_buf2, "OK", 50);
		delay_ms(4000);
		//3.����·����(�������ӵ��ֻ��ȵ�)
		esp8266_reset_ref();
		while (esp8266_send_data(at_buf3, "WIFI GOT IP", 800))
		{
			oled_dis_str(2, 2, "wifi net");
			delay_ms(1000);
			oled_dis_clear();
		}
		delay_ms(1000);
		//	esp8266_reset_ref();
		//	esp8266_send_data(at_buf4, "OK", 300);
		//	delay_ms(1000);
		//4.���ӵ�sever
		esp8266_reset_ref();
		while (esp8266_send_data(at_buf5, "OK", 200))
		{
			oled_dis_str(2, 2, "wifi sever");
			delay_ms(1000);
			oled_dis_clear();
		}
		delay_ms(1000);
		//5.����͸��ģʽ
		esp8266_reset_ref();
		esp8266_send_data(at_buf6, "OK", 200);
		delay_ms(1000);
		//6.��ʼ͸��
		esp8266_reset_ref();
		esp8266_send_data(at_buf7, "OK", 50);
		esp8266_reset_ref();
	}
	
//	if (WIFI_STA_TCP == trans_mode)	//TCP Client ģʽ����
//	{
//		printf("\r\n %s %d \r\n", at_buf3, sizeof(at_buf3));
//		printf("\r\n %s %d \r\n", at_buf5, sizeof(at_buf5));
//	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_uart_exit
//	����˵��: ESP8266 �˳�͸��ģʽ
//	��    ��: ��
//	�� �� ֵ: 0
//	��    ��: 2021-07-07
//  ��    ע: ���͵����ݲ�����"\r\n"
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void esp8266_uart_exit(void)
{
	uint8_t ex_buf[] = "+++";
	
	uart_send_serial_bytes(ESP8266_COMM_NO, ex_buf, 3);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_test
//	����˵��: ESP8266 ģ����,�жϳ�ʼ���Ƿ�ɹ�
//	��    ��: ��
//	�� �� ֵ: 0
//	��    ��: 2021-06-28
//  ��    ע: ����ATָ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t esp8266_test(void)
{
	uint8_t count = 30;
	uint16_t i = 0;
	uint8_t ret_val = 0xFF;
	
	g_t_wifi.wifi_comm_mode = ESP_RX_MODE;	//����ģʽ
	
	esp8266_send_data("AT", "OK", 20);
//	delay_ms(3000);
//	esp8266_send_data("AT+RST", "OK", 1000);
/*	
	while (count--)
	{
		esp8266_send_at_cmd("AT");
		delay_ms(101);
		
//		if (g_t_wifi.wifi_rx_length)
//		{
//			printf("wifi_length is %d \r\n", g_t_wifi.wifi_rx_length);
//			while (i <= g_t_wifi.wifi_rx_length)
//			{
//				printf("wifi recv: %C \r\n", g_t_wifi.wifi_rx_buff[i++]);
//			}
//			printf("\r\n wifi rx flag: %d \r\n\r\n", g_t_wifi.wifi_comm_mode);
//			printf("\r\n");
//		}
//		else 
//		{
//			printf("no recv -- ble_length is %d \r\n", g_t_wifi.wifi_rx_length);
//		}
		
		if (esp8266_check_at_ack("OK"))
		{
			printf("wifi_length is %d \r\n", g_t_wifi.wifi_rx_length);
			printf("count is %d \r\n", count);
//			for (i = 0; i < 4; i++)	
				printf("wifi recv: %c \r\n", g_t_wifi.wifi_rx_buff[0]);
			printf("wifi recv: %c \r\n", g_t_wifi.wifi_rx_buff[1]);
			oled_dis_str(2, 2, "wifi ok!");
			
			printf("\r\n wifi rx flag: %d \r\n\r\n", g_t_wifi.wifi_comm_mode);
			
			for (i = 0; i < g_t_wifi.wifi_rx_length; i++)	
				printf("***wifi recv: 0x%02x \r\n", g_t_wifi.wifi_rx_buff[i]);
			
			ret_val = 0;
			
			return ret_val;
//			break;
		}
		
 		if (count == 0)
		{
			ret_val = 1;
//			g_t_ble.ble_buff[0] = 'A';
			oled_dis_str(2, 6, "wifi err!");
//			oled_dis_str(2, 2, &g_t_ble.ble_rx_buff[0]);
//			oled_dis_str(16, 2, &g_t_ble.ble_rx_buff[1]);
//			oled_dis_str(2, 4, "count == 0");
			
			return ret_val;
		}
	}
*/
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: esp8266_reset_ref
//	����˵��: wifi ��ز����ָ���ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-28
//  ��    ע: �����ָ�Ĭ��״̬
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void esp8266_reset_ref(void)
{
	uint16_t i;
	
	if (g_t_wifi.wifi_comm_mode == ESP_RX_OK)
	{
		g_t_wifi.wifi_comm_mode = ESP_RX_MODE;	//����ģʽ
		
		g_t_wifi.wifi_rx_length = 0;
		for (i = 0; i < ESP_RX_LEN; i++)
		{
			g_t_wifi.wifi_rx_buff[i] = 0x00;
		}
	}
}

#endif /* __PRJ_STM32F40X_DRVESP8266_C__ */
