//--------------------------------------------------------------------------------------------------------
//
//	ģ������ : uart�շ�����
//	�ļ����� : drvuart.c
//	��    �� : V1.0
//	˵    �� : 
//				(1) drvuart.c �շ�,��ѯ+�ж�
//				(2) 
//				(3) 
//				(4) 
//
//	�޸ļ�¼ :
//		�汾��     ����        ����     ˵��
//		V1.0    2021-05-25   ����AI  ��ʽ����
//		V1.1    
//		V1.2	
//		V1.3	
//
//	Copyright (C), 2020-2030, ΢�Ź��ںš���TECHTIMES
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

//��Ҫ���Micro LIB һ��ʹ��(��ѡ΢��)
int fputc(int ch, FILE *f)
{      
//	while((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������   
//    USART1->DR = (uint8_t)ch;      
	
	 uart_send_byte(DEBUG_COMM_NO, (uint8_t)ch);
//	uart_send_byte(4, (uint8_t)ch);

//	uart_send_serial_bytes(1, (uint8_t *)&ch, 1);
	
	return ch;
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_gpio_init
//	����˵��: ����GPIO��ʼ��
//	��    ��: 	uart_chl�����ڱ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-25
//  ��    ע: ����GPIO��ʼ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_gpio_init(uint8_t uart_chl)
{
    GPIO_InitTypeDef gpio_config_init;

    if (uart_chl == 1)
    {
        RCC_AHB1PeriphClockCmd(RCC_PCLK_USART1_GPIO, ENABLE);	//ʹ��UART1 GPIOʱ��,AHB1����
		
		//����1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(USART1_TXD_PORT, GPIO_PinSource9, GPIO_AF_USART1); //GPIOA9����ΪUSART1
		GPIO_PinAFConfig(USART1_RXD_PORT, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10����ΪUSART1

		gpio_config_init.GPIO_Pin 	= USART1_TXD_IO; //PA9
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART1_TXD_PORT, &gpio_config_init);//��ʼ��GPIOA9

		gpio_config_init.GPIO_Pin 	= USART1_RXD_IO;//PA10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//���ù��ܱ�������
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART1_RXD_PORT, &gpio_config_init);//��ʼ��GPIOA10
    }
   else if (uart_chl == 2)
    {
       RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	//ʹ��UART1 GPIOʱ��,APB2����
		
		//����1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(USART2_TXD_PORT, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2����ΪUSART1
		GPIO_PinAFConfig(USART2_RXD_PORT, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3����ΪUSART1

		gpio_config_init.GPIO_Pin 	= USART2_TXD_IO; //PA2
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART2_TXD_PORT, &gpio_config_init);//��ʼ��GPIOA2

		gpio_config_init.GPIO_Pin 	= USART2_RXD_IO;//PA3
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//���ù���
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_DOWN;
		GPIO_Init(USART2_RXD_PORT, &gpio_config_init);//��ʼ��GPIOA3
    }
    else if (uart_chl == 3)
    {
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	//ʹ��UART3 GPIOʱ��,AHB1����
		
		//����1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(USART3_TXD_PORT, GPIO_PinSource10, GPIO_AF_USART3); //GPIOB10����ΪUSART3
		GPIO_PinAFConfig(USART3_RXD_PORT, GPIO_PinSource11, GPIO_AF_USART3); //GPIOB11����ΪUSART3

		gpio_config_init.GPIO_Pin 	= USART3_TXD_IO; //PB10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART3_TXD_PORT, &gpio_config_init);//��ʼ��GPIOB10

		gpio_config_init.GPIO_Pin 	= USART3_RXD_IO;//PB11
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(USART3_RXD_PORT, &gpio_config_init);//��ʼ��GPIOB11
    }
     else if (uart_chl == 4)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//ʹ��UART4 GPIOʱ��,AHB1����
		
		//����4��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(UART4_TXD_PORT, GPIO_PinSource10, GPIO_AF_UART4); //GPIOC10����ΪUART4
		GPIO_PinAFConfig(UART4_RXD_PORT, GPIO_PinSource11, GPIO_AF_UART4); //GPIOC11����ΪUART4

		gpio_config_init.GPIO_Pin 	= UART4_TXD_IO; //PC10
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART4_TXD_PORT, &gpio_config_init);//��ʼ��GPIO PC10

		gpio_config_init.GPIO_Pin 	= UART4_RXD_IO;//PC11
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART4_RXD_PORT, &gpio_config_init);//��ʼ��GPIO PC11
    }
     else if (uart_chl == 5)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	//ʹ��UART5 GPIOʱ��,AHB1����
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//ʹ��UART5 GPIOʱ��,AHB1����

		//����4��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(UART5_TXD_PORT, GPIO_PinSource12, GPIO_AF_UART5); //GPIOC12����ΪUART5
		GPIO_PinAFConfig(UART5_RXD_PORT, GPIO_PinSource2,  GPIO_AF_UART5); //GPIOD2����ΪUART5

		gpio_config_init.GPIO_Pin 	= UART5_TXD_IO; //PC12
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
		gpio_config_init.GPIO_OType = GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART5_TXD_PORT, &gpio_config_init);//��ʼ��GPIO PC12

		gpio_config_init.GPIO_Pin 	= UART5_RXD_IO;//PD2
		gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 	= GPIO_Mode_AF;	//�����������
//		gpio_config_init.GPIO_OType = GPIO_OType_PP;
//		gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
		GPIO_Init(UART5_RXD_PORT, &gpio_config_init);//��ʼ��GPIO PD2
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_config
//	����˵��: �������ò�����ʼ��
//	��    ��: 	uart_chl�����ڱ��
//              uart_bound��������
//	�� �� ֵ: ��
//	��    ��: 2020-03-25
//  ��    ע: ���ڱ�Ŵ�1-5(UART1-5),�����ж�ʹ����Nvic������������
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_config(uint8_t uart_chl, uint32_t uart_bound)
{
    USART_InitTypeDef uart_config_init;

    if (uart_chl == 1)
    {
		RCC_APB2PeriphClockCmd(RCC_PCLK_USART1, ENABLE);	//APB2
		
		uart_config_init.USART_BaudRate              = uart_bound;	//���ڲ�����
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//һ��ֹͣλ
		uart_config_init.USART_Parity                = USART_Parity_No;	//����żУ��λ
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//��Ӳ������������
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART1, &uart_config_init); //��ʼ������1
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�(���ڲ�����ʼ��֮����ܿ����ж�)
		USART_Cmd(USART1, ENABLE); 		//ʹ�ܴ���1 
    }
    else if (uart_chl == 2)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_USART2, ENABLE);	//APB1
		
		uart_config_init.USART_BaudRate             = uart_bound;//���ڲ�����
		uart_config_init.USART_WordLength           = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		uart_config_init.USART_StopBits             = USART_StopBits_1;//һ��ֹͣλ
		uart_config_init.USART_Parity               = USART_Parity_No;//����żУ��λ
		uart_config_init.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;//��Ӳ������������
		uart_config_init.USART_Mode                 = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART2, &uart_config_init); //��ʼ������2
		// USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
		USART_Cmd(USART2, ENABLE); 		//ʹ�ܴ���2 
    }
    else if (uart_chl == 3)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_USART3, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//���ڲ�����
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//һ��ֹͣλ
		uart_config_init.USART_Parity                = USART_Parity_No;	//����żУ��λ
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//��Ӳ������������
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART3, &uart_config_init); //��ʼ������1
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�(���ڲ�����ʼ��֮����ܿ����ж�)
		USART_Cmd(USART3, ENABLE); 		//ʹ�ܴ���3 
    }
  else if (uart_chl == 4)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_UART4, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//���ڲ�����
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//һ��ֹͣλ
		uart_config_init.USART_Parity                = USART_Parity_No;	//����żУ��λ
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//��Ӳ������������
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(UART4, &uart_config_init); //��ʼ������4
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�(���ڲ�����ʼ��֮����ܿ����ж�)
		USART_Cmd(UART4, ENABLE); 		//ʹ�ܴ���4 
    }
      else if (uart_chl == 5)
    {
		RCC_APB1PeriphClockCmd(RCC_PCLK_UART5, ENABLE);
		
		uart_config_init.USART_BaudRate              = uart_bound;	//���ڲ�����
		uart_config_init.USART_WordLength            = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
		uart_config_init.USART_StopBits              = USART_StopBits_1;	//һ��ֹͣλ
		uart_config_init.USART_Parity                = USART_Parity_No;	//����żУ��λ
		uart_config_init.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;	//��Ӳ������������
		uart_config_init.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(UART5, &uart_config_init); //��ʼ������5
		// USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�(���ڲ�����ʼ��֮����ܿ����ж�)
		USART_Cmd(UART5, ENABLE); 		//ʹ�ܴ���5 
    }

}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_irq_set
//	����˵��: �����жϳ�ʼ������
//	��    ��: 	uart_chl�����ڱ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-27
//  ��    ע: ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
static void uart_irq_set(uint8_t uart_chl)
{
	NVIC_InitTypeDef uart_nvic_config;

	if (uart_chl == 1)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART1_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//��ռ���ȼ�3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//�����ȼ�3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&uart_nvic_config);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//�������ڽ����ж�
    }
    else if (uart_chl == 2)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART2_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 3 ;//��ռ���ȼ�3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//�����ȼ�3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&uart_nvic_config);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    }
    else if (uart_chl == 3)
    {
        uart_nvic_config.NVIC_IRQChannel 					= USART3_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//��ռ���ȼ�3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 2;		//�����ȼ�3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&uart_nvic_config);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    }
    else if (uart_chl == 4)
    {
        uart_nvic_config.NVIC_IRQChannel 					= UART4_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 2 ;//��ռ���ȼ�3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//�����ȼ�3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&uart_nvic_config);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	//�������ڽ����ж�
    }
    else if (uart_chl == 5)
    {
        uart_nvic_config.NVIC_IRQChannel 					= UART5_IRQn;
		uart_nvic_config.NVIC_IRQChannelPreemptionPriority 	= 3 ;//��ռ���ȼ�3
		uart_nvic_config.NVIC_IRQChannelSubPriority 		= 3;		//�����ȼ�3
		uart_nvic_config.NVIC_IRQChannelCmd 				= ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&uart_nvic_config);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    }
    else 
    {
//        printf("error! \r\n");
    }
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_init
//	����˵��: ����G���ò�����ʼ��
//	��    ��: 	uart_chl�����ڱ��
//              uart_bound��������
//	�� �� ֵ: ��
//	��    ��: 2020-03-25
//  ��    ע: ͳһ���õײ�ӿ�(�����֣�GPIO + UART + NVIC)
//	��    ��: by ����AI
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
//	�� �� ��: uart_send_byte
//	����˵��: ���ڷ���һ�ֽ�����
//	��    ��: 	uart_chl�����ڱ��
//              send_byte�����͵�����
//	�� �� ֵ: ��
//	��    ��: 2020-03-27
//  ��    ע: ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_send_byte(uint8_t uart_chl, uint8_t send_byte)
{
	if (uart_chl == 1)
    {
	#if 0
		while((USART1->SR & 0X40) == 0);//ѭ������,ֱ���������   
		USART1->DR = (uint8_t)send_byte; 
	#else
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//�ȴ����ڷ��ͱ�־λ��λ

		USART_SendData(USART1, (uint8_t)send_byte);	//��������

	#endif
    }
    else if (uart_chl == 2)
    {
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);	//�ȴ����ڷ��ͱ�־λ��λ

		USART_SendData(USART2, (uint8_t)send_byte);	//��������
    }
    else if (uart_chl == 3)
    {
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);	//�ȴ����ڷ��ͱ�־λ��λ

		USART_SendData(USART3, (uint8_t)send_byte);	//��������
    }
    else if (uart_chl == 4)
    {
		while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);	//�ȴ����ڷ��ͱ�־λ��λ

		USART_SendData(UART4, (uint8_t)send_byte);	//��������
    }
    else if (uart_chl == 5)
    {
		while(USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET);	//�ȴ����ڷ��ͱ�־λ��λ

		USART_SendData(UART5, (uint8_t)send_byte);	//��������
    }
    else 
    {
//        printf("error! \r\n");
    }

	return 0;
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_send_serial_bytes
//	����˵��: ���ڷ��Ͷ��ֽ�����
//	��    ��: 	uart_chl�����ڱ��
//              pbdata�����͵����ݵ�ַ
//				length���������ݵĳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-28
//  ��    ע: ��
//	��    ��: by ����AI
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
//	�� �� ��: uart_recv_byte
//	����˵��: ���ڽ���һ�ֽ�����
//	��    ��: 	uart_chl�����ڱ��
//	�� �� ֵ: recv_byte�����յ�����
//	��    ��: 2020-03-28
//  ��    ע: ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_recv_byte(uint8_t uart_chl)
{
	uint8_t recv_byte = 0;

	if (uart_chl == 1)
    {
	#if 0
		while((USART1->SR & 0X20) == 0);//ѭ������,ֱ���������   
		recv_byte = USART1->DR; 
	#else
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);	//�ȴ����ڽ��ձ�־λ��λ
		recv_byte = USART_ReceiveData(USART1);	//��������

	#endif
    }
    else if (uart_chl == 2)
    {
		while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);	//�ȴ����ڽ��ձ�־λ��λ
		recv_byte = USART_ReceiveData(USART2);	//��������
    }
    else if (uart_chl == 3)
    {
		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET);	//�ȴ����ڽ��ձ�־λ��λ
		recv_byte = USART_ReceiveData(USART3);	//��������
    }
    else if (uart_chl == 4)
    {
		while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);	//�ȴ����ڽ��ձ�־λ��λ
		recv_byte = USART_ReceiveData(UART4);	//��������
    }
    else if (uart_chl == 5)
    {
		while (USART_GetFlagStatus(UART5, USART_FLAG_RXNE) == RESET);	//�ȴ����ڽ��ձ�־λ��λ
		recv_byte = USART_ReceiveData(UART5);	//��������
    }
    else 
    {
//        printf("error! \r\n");
    }
	
	return recv_byte;
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_recv_serial_bytes
//	����˵��: ���ڽ��ն��ֽ�����
//	��    ��: 	uart_chl�����ڱ��
//              pbdata�����͵����ݵ�ַ
//				length���������ݵĳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-28
//  ��    ע: ��
//	��    ��: by ����AI
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
//	�� �� ��: USART1_IRQHandler
//	����˵��: �����жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-05-25
//  ��    ע: ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t uart_rx_buf[10];	
uint8_t uart_rx_len = 0;
void USART1_IRQHandler(void)
{
	uint8_t i = 0;
	
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//�������жϱ�־λ
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

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	//������ڽ����жϱ�־
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: USART3_IRQHandler
//	����˵��: ����3�жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-09
//  ��    ע: ����HC05ģ��Ľ�������
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void)
{
	uint8_t i = 0;
	uint8_t ret_dat;
	
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)	//�������жϱ�־λ
	{
//		TIM_SetCounter(TIM2, 0);	//����������
//		TIM_Cmd(TIM2, ENABLE);	//��û�н������,�������
		
		ret_dat = USART_ReceiveData(USART3);
		
		if (g_t_ble.comm_mode != BLE_RX_OK)	//ble����û�����
		{
			if (g_t_ble.ble_rx_length < BLE_RX_LEN)	//ble��������С�����Ƴ���
			{
				TIM_SetCounter(TIM2, 0);	//����������
				if (g_t_ble.comm_mode == BLE_RX_MODE)
				{
					TIM_Cmd(TIM2, ENABLE);	//��û�н������,�������
				}
				g_t_ble.ble_rx_buff[g_t_ble.ble_rx_length++] = ret_dat;
				
	//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//������ڽ����жϱ�־
			}
			else 
			{
				g_t_ble.comm_mode = BLE_RX_OK;	//��������������,ǿ��ֹͣ
			}
		}
		
//		g_t_ble.ble_length++;
//		oled_dis_str(2, 2, &g_t_ble.ble_buff[0]);
//		oled_dis_str(16, 2, &g_t_ble.ble_buff[1]);
//		oled_dis_num(2, 6, g_t_ble.ble_length, 3, 16, 0x80);
//		oled_dis_num(63, 6, g_t_ble.ble_buff[1], 3, 16, 0x80);

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//������ڽ����жϱ�־
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: UART4_IRQHandler
//	����˵��: ����4�жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-28
//  ��    ע: ����WIFIģ��Ľ�������
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
void UART4_IRQHandler(void)
{
/*
	uint8_t i = 0;
	
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)	//�������жϱ�־λ
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

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);	//������ڽ����жϱ�־
	}
*/
	uint8_t i = 0;
	uint8_t ret_dat;
	
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)	//�������жϱ�־λ
	{
//		TIM_SetCounter(TIM4, 0);	//����������
//		TIM_Cmd(TIM4, ENABLE);	//��û�н������,�������
		
		ret_dat = USART_ReceiveData(UART4);
		
		if (g_t_wifi.wifi_comm_mode != ESP_RX_OK)	//esp8266����û�����
		{
			if (g_t_wifi.wifi_rx_length < ESP_RX_LEN)	//esp8266��������С�����Ƴ���
			{
				TIM_SetCounter(TIM4, 0);	//����������
				if (g_t_wifi.wifi_comm_mode == ESP_RX_MODE)
				{
					TIM_Cmd(TIM4, ENABLE);	//��û�н������,�������
				}
				g_t_wifi.wifi_rx_buff[g_t_wifi.wifi_rx_length++] = ret_dat;
				
	//			USART_ClearITPendingBit(USART3, USART_IT_RXNE);	//������ڽ����жϱ�־
			}
			else 
			{
				g_t_wifi.wifi_comm_mode = ESP_RX_OK;	//��������������,ǿ��ֹͣ
			}
		}
		
//		g_t_wifi.wifi_rx_length++;
//		oled_dis_str(2, 2, &g_t_wifi.wifi_rx_buff[0]);
//		oled_dis_str(16, 2, &g_t_wifi.wifi_rx_buff[1]);
//		oled_dis_num(2, 6, g_t_wifi.wifi_rx_length, 3, 16, 0x80);
//		oled_dis_num(63, 6, g_t_wifi.wifi_rx_buff[1], 3, 16, 0x80);

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);	//������ڽ����жϱ�־
	}
}

void UART5_IRQHandler(void)
{
	uint8_t i = 0;
	
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)	//�������жϱ�־λ
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

		USART_ClearITPendingBit(UART5, USART_IT_RXNE);	//������ڽ����жϱ�־
	}
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: uart_test
//	����˵��: �����շ����Ժ���
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-28
//  ��    ע: ��
//	��    ��: by ����AI
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
