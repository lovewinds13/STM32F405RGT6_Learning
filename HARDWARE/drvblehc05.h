#ifndef __DRVBLEHC05_H__
#define __DRVBLEHC05_H__

#include "sys.h"

//GPIO 部分
#define RCC_PCLK_BLE_GPIO			RCC_AHB1Periph_GPIOB
#define PORT_BLE_IN					GPIOB
#define BLE_EN_IO					GPIO_Pin_12

#define BLE_EN_1()		(GPIO_SetBits(PORT_BLE_IN, BLE_EN_IO))
#define BLE_EN_0()		(GPIO_ResetBits(PORT_BLE_IN, BLE_EN_IO))

//BLE 部分
#define BLE_COMM_NO		3

#define EN_IO_MODE		0	//(HC05按键触发?)

#define BLE_DE_MODE 	0x00
#define BLE_TX_MODE 	0x01
#define BLE_RX_MODE 	0x02
#define BLE_RX_OK		0x03

#define BLE_TX_LEN 		100
#define BLE_RX_LEN 		100

extern uint8_t ble_rx_buff[BLE_RX_LEN];	
extern uint8_t ble_tx_buff[BLE_TX_LEN];

typedef struct 
{
	uint8_t comm_mode;
	uint8_t *ble_rx_buff;
	uint16_t ble_rx_length;
	uint8_t *ble_tx_buff;
	uint16_t ble_tx_length;
}BLE_HC05_T;

//函数声明部分
extern uint8_t ble_init(void);
extern uint8_t ble_detect(void);
extern void ble_cmd_test(void);
extern void ble_reset_ref(void);

#endif	//__DRVBLEHC05_H__
