#ifndef __PRJ_STM32F40X_DRVTIMER_C__
#define __PRJ_STM32F40X_DRVTIMER_C__

#include "drvtimer.h"
#include "drvled.h"
#include "drvoled.h"
#include "drvblehc05.h"
#include "drvesp8266.h"

//#define DRIVER_TIMER_REG	//����Ĵ�����ʽ

extern BLE_HC05_T g_t_ble;
extern WIFI_ESP_T g_t_wifi;

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: timer_config
//	����˵��: ��ʱ����������
//	��    ��: 	timer_no����ʱ�����
//				timer_arr���Զ���װֵ
//				timer_psc��ʱ�ӷ�Ƶϵ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-14
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void timer_config(uint8_t timer_no, uint16_t timer_arr, uint16_t timer_psc)
{
#ifdef DRIVER_TIMER_REG	//�Ĵ�����ʽ
	if (timer_no == 3)
	{
		RCC->APB1ENR	|= 1 << 1;	//��ʱ��3ʱ��ʹ��(APB1)
		TIM3->ARR		 = timer_arr;	//�Զ���װ�ؼ�������ֵ
		TIM3->PSC		 = timer_psc;	//Ԥ��Ƶ����ֵ
		TIM3->DIER		|= 1 << 0;	//�жϸ���ʹ��
		TIM3->CR1	 	|= 0x01;	//ʹ�ܶ�ʱ��3
	}
#else	
	TIM_TimeBaseInitTypeDef timer_init_config;
	
	if (timer_no == 2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//�Զ���װ�ؼĴ�����ֵ
		timer_init_config.TIM_Prescaler 			= timer_psc;	//ʱ��Ƶ�ʳ���Ԥ��Ƶ��ֵ
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//ʱ�ӷָ�
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//���ϼ�����ʽ
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//�߼���ʱ��ʹ��
		TIM_TimeBaseInit(TIM2, &timer_init_config);
	}
	if (timer_no == 3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//�Զ���װ�ؼĴ�����ֵ
		timer_init_config.TIM_Prescaler 			= timer_psc;	//ʱ��Ƶ�ʳ���Ԥ��Ƶ��ֵ
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//ʱ�ӷָ�
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//���ϼ�����ʽ
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//�߼���ʱ��ʹ��
		TIM_TimeBaseInit(TIM3, &timer_init_config);
	}
	if (timer_no == 4)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		
		timer_init_config.TIM_Period 				= timer_arr;	//�Զ���װ�ؼĴ�����ֵ
		timer_init_config.TIM_Prescaler 			= timer_psc;	//ʱ��Ƶ�ʳ���Ԥ��Ƶ��ֵ
		timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//ʱ�ӷָ�
		timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//���ϼ�����ʽ
		//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//�߼���ʱ��ʹ��
		TIM_TimeBaseInit(TIM4, &timer_init_config);
	}
#endif	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: timer_nvic_config
//	����˵��: ��ʱ��NVIC��������
//	��    ��: 	timer_no����ʱ�����
//	�� �� ֵ: ��
//	��    ��: 2020-04-14
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
#ifndef DRIVER_TIMER_REG
static void timer_nvic_config(uint8_t timer_no)
{
	NVIC_InitTypeDef nvic_init_config;
	
	if (timer_no == 2)
	{
		//�ж����ȼ� NVIC ����
		nvic_init_config.NVIC_IRQChannel 						= TIM2_IRQn; //TIM2 �ж�
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 1; //��ռ���ȼ� 1 ��
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 1; //�����ȼ� 1 ��
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ ͨ����ʹ��
		NVIC_Init(&nvic_init_config); //��ʼ�� NVIC �Ĵ���
		
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//��������ж�
	}

	if (timer_no == 3)
	{
		//�ж����ȼ� NVIC ����
		nvic_init_config.NVIC_IRQChannel 						= TIM3_IRQn; //TIM3 �ж�
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 3; //��ռ���ȼ� 2 ��
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 3; //�����ȼ� 3 ��
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ ͨ����ʹ��
		NVIC_Init(&nvic_init_config); //��ʼ�� NVIC �Ĵ���
		
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//��������ж�
	}
	
	if (timer_no == 4)
	{
		//�ж����ȼ� NVIC ����
		nvic_init_config.NVIC_IRQChannel 						= TIM4_IRQn; //TIM4 �ж�
		nvic_init_config.NVIC_IRQChannelPreemptionPriority 		= 0; //��ռ���ȼ� 1 ��
		nvic_init_config.NVIC_IRQChannelSubPriority 			= 1; //�����ȼ� 1 ��
		nvic_init_config.NVIC_IRQChannelCmd 					= ENABLE; //IRQ ͨ����ʹ��
		NVIC_Init(&nvic_init_config); //��ʼ�� NVIC �Ĵ���
		
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//��������ж�
	}
}
#endif

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: timer_init
//	����˵��: ��ʱ����ʼ������
//	��    ��: 	timer_no����ʱ�����
//	�� �� ֵ: ��
//	��    ��: 2020-04-14
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------

void timer_init(uint8_t timer_no)
{
#ifdef DRIVER_TIMER_REG
	
#else
	if (timer_no == 2)
	{
		timer_config(timer_no, 100-1, 8400-1);//10ms�ж�һ��
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM2, ENABLE);//������ʱ��
	}
	
	if (timer_no == 3)
	{
		timer_config(timer_no, 4999, 8399);//��������5000 = 500ms;����Ƶ��:84M/8400=10KHz
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM3, ENABLE);//������ʱ��
	}
	
	if (timer_no == 4)
	{
		timer_config(timer_no, 1000-1, 8400-1);//100ms�ж�һ��
		timer_nvic_config(timer_no);
		
		TIM_Cmd(TIM4, ENABLE);//������ʱ��TIM4
	}
#endif	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: TIM2_IRQHandler
//	����˵��: ��ʱ��2�жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-09
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		g_t_ble.comm_mode = BLE_RX_OK;	//10msû�����յ����ݱ�ʾ���
//		g_t_ble.comm_mode = 99;
//		Bsp_LedToggle(0);
	}
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	TIM_Cmd(TIM2, DISABLE);	//ֹͣ����
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: TIM3_IRQHandler
//	����˵��: ��ʱ��3�жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-05-27
//  ��    ע: 
//	��    ��: by ����AI
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
//	�� �� ��: TIM4_IRQHandler
//	����˵��: ��ʱ��4�жϷ�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2021-06-28
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		g_t_wifi.wifi_comm_mode = ESP_RX_OK;	//100msû�����յ����ݱ�ʾ���
	}
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	
	TIM_Cmd(TIM4, DISABLE);	//ֹͣ����
}


#endif	//__PRJ_STM32F40X_DRVTIMER_C__
