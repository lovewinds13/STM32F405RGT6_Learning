#ifndef __DRVUART_H__
#define __DRVUART_H__

#include "sys.h"

#define DEBUG_COMM_NO	1

//GPIO 部分
#define RCC_PCLK_USART1_GPIO		RCC_AHB1Periph_GPIOA
#define USART1_TXD_IO				GPIO_Pin_9
#define USART1_RXD_IO				GPIO_Pin_10
#define USART1_TXD_PORT				GPIOA
#define USART1_RXD_PORT				GPIOA

#define RCC_PCLK_USART2_GPIO		RCC_AHB1Periph_GPIOA
#define USART2_TXD_IO				GPIO_Pin_2
#define USART2_RXD_IO				GPIO_Pin_3
#define USART2_TXD_PORT				GPIOA
#define USART2_RXD_PORT				GPIOA

#define RCC_PCLK_USART3_GPIO		RCC_AHB1Periph_GPIOB
#define USART3_TXD_IO				GPIO_Pin_10
#define USART3_RXD_IO				GPIO_Pin_11
#define USART3_TXD_PORT				GPIOB
#define USART3_RXD_PORT				GPIOB

//UART4
#define RCC_PCLK_UART4_GPIO			RCC_AHB1Periph_GPIOC
#define UART4_TXD_IO				GPIO_Pin_10
#define UART4_RXD_IO				GPIO_Pin_11
#define UART4_TXD_PORT				GPIOC
#define UART4_RXD_PORT	            GPIOC

#define RCC_PCLK_UART5_GPIO		(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD)
#define UART5_TXD_IO				GPIO_Pin_12
#define UART5_RXD_IO				GPIO_Pin_2
#define UART5_TXD_PORT				GPIOC
#define UART5_RXD_PORT	            GPIOD

//UART 部分
#define RCC_PCLK_USART1				RCC_APB2Periph_USART1
#define RCC_PCLK_USART2				RCC_APB1Periph_USART2
#define RCC_PCLK_USART3				RCC_APB1Periph_USART3
#define RCC_PCLK_UART4				RCC_APB1Periph_UART4
#define RCC_PCLK_UART5				RCC_APB1Periph_UART5

//函数声明部分
extern void uart_init(uint8_t uart_chl, uint32_t uart_bound);
extern uint8_t uart_send_byte(uint8_t uart_chl, uint8_t send_byte);
extern uint8_t uart_send_serial_bytes(uint8_t uart_chl, uint8_t *pbdata, uint16_t length);
extern uint8_t uart_recv_byte(uint8_t uart_chl);
extern uint8_t uart_recv_serial_bytes(uint8_t uart_chl, uint8_t *pbdata, uint16_t length);

extern void uart_test(void);

#endif	//__DRVUART_H__

