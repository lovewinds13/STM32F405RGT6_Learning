#ifndef __DRVESP8266_H__
#define __DRVESP8266_H__

#include "sys.h"

//GPIO 部分
#define RCC_PCLK_ESP8266_GPIO			RCC_AHB1Periph_GPIOB
#define PORT_ESP8266_IN					GPIOB
#define ESP8266_EN_IO					GPIO_Pin_12

#define ESP8266_EN_1()		(GPIO_SetBits(PORT_BLE_IN, BLE_EN_IO))
#define ESP8266_EN_0()		(GPIO_ResetBits(PORT_BLE_IN, BLE_EN_IO))

//WIFI 部分
#define ESP8266_COMM_NO		4

#define ESP_DE_MODE 	0x00
#define ESP_TX_MODE 	0x01
#define ESP_RX_MODE 	0x02
#define ESP_RX_OK		0x03

#define ESP_TX_LEN 		200
#define ESP_RX_LEN 		400

#define WIFI_STA_TCP_C	0x01
#define WIFI_STA_TCP_S	0x01
#define WIFI_STA_UDP	0x03

extern uint8_t wifi_rx_buf[ESP_RX_LEN];	
extern uint8_t wifi_tx_buf[ESP_TX_LEN];

typedef struct 
{
	uint8_t wifi_comm_mode;
	uint8_t *wifi_rx_buff;
	uint16_t wifi_rx_length;
	uint8_t *wifi_tx_buff;
	uint16_t wifi_tx_length;
}WIFI_ESP_T;

//typedef struct 
//{
//	uint8_t *wifi_sta_mode;
//	uint8_t *wifi_sta_ssid;
//	uint8_t *wifi_sta_pd;
//	uint8_t *wifi_sta_ip;
//	uint8_t *wifi_sta_port;
//}WIFI_STA_PRO_T;

extern void esp8266_init(void);
extern void esp8266_send_at_cmd(const char *at_cmd);
extern uint8_t esp8266_test(void);
extern void esp8266_reset_ref(void);
extern uint8_t* esp8266_check_at_ack(uint8_t *ptr_cmd);
extern uint8_t esp8266_send_data(uint8_t *dat_buf, uint8_t *ans_ack, uint16_t wait_time);
extern uint8_t esp8266_uart_trans(uint8_t trans_mode);
extern void esp8266_uart_exit(void);

#endif	//__DRVESP8266_H__
