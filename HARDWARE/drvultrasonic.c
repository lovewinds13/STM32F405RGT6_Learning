//---------------------------------------------------------------------------------------------------------------------------------------------
//ƽ    ̨:				STM32F10X
//��    ��:    		 	drultrasonic.c
//��    ��:       		����AI
//��� ��:   			Vxxx
//�ļ��汾:   			V1.0.0
//��   ��:      		2021��04��11��
//˵   ��:      	 	������ģ�� HC-SR04 ����ʵ��
//----------------------------------------------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVULTRASONIC_C__
#define __PRJ_STM32F40X_DRVULTRASONIC_C__

#include "delay.h"
#include "drvtimer.h"
#include "drvuart.h"
#include "drvultrasonic.h"
#include "stdio.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: hcsr04_gpio_init
//	����˵��: ������ģ��GPIO����
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2021-04-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void hcsr04_gpio_init(void)
{
	GPIO_InitTypeDef gpio_initx;
	  
	RCC_AHB1PeriphClockCmd(RCC_PCLK_ULT_GPIO, ENABLE);	//ʹ��GPIOBʱ��
	
	//PA0 ��Ϊ�����������                         
	gpio_initx.GPIO_Pin		= ULT_TRIG_PIN;	//TRIG����
	gpio_initx.GPIO_Speed 	= GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	gpio_initx.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_initx.GPIO_OType 	= GPIO_OType_PP;	//�������
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_UP; 
	GPIO_Init(PORT_ULT_GPIO, &gpio_initx);	
	
		//PA1 ��Ϊ����������                         
	gpio_initx.GPIO_Pin		= ULT_ECHO_PIN;	//ECHO����
	gpio_initx.GPIO_Speed 	= GPIO_Speed_50MHz;
	gpio_initx.GPIO_Mode 	= GPIO_Mode_IN;//��������	
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_DOWN;		//������������
	GPIO_Init(PORT_ULT_GPIO, &gpio_initx);
	
	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);

#if 1	//����һ����ʱ������ʱ��
	TIM_TimeBaseInitTypeDef timer_init_config;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	timer_init_config.TIM_Period 				= 65535;	//�Զ���װ�ؼĴ�����ֵ
	timer_init_config.TIM_Prescaler 			= 84;	//ʱ��Ƶ�ʳ���Ԥ��Ƶ��ֵ(����һ��1us)
	timer_init_config.TIM_ClockDivision 		= TIM_CKD_DIV1;	//ʱ�ӷָ�
	timer_init_config.TIM_CounterMode 			= TIM_CounterMode_Up;	//���ϼ�����ʽ
	//timer_init_config.TIM_RepetitionCounter 	= 0x0000;//�߼���ʱ��ʹ��
	TIM_TimeBaseInit(TIM4, &timer_init_config);
	
//	TIM_SetCounter(TIM3, 0);	//��ʱ���ļ�����ֵ����Ϊ0
	TIM_Cmd(TIM4, ENABLE);	//��ʱ��ʹ��
#endif
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: hcsr04_start
//	����˵��: ������ģ����������
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: ����һ������10us�ĸߵ�ƽ
//	��    ��: 2021-04-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void hcsr04_start(void)
{
//	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
//	delay_us(2);
	GPIO_SetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
	delay_us(12);	//�ߵ�ƽʱ�����10us
	GPIO_ResetBits(PORT_ULT_GPIO, ULT_TRIG_PIN);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: hcsr04_read_data
//	����˵��: ������ģ���ȡ��������
//	��    ��: ��
//	�� �� ֵ: tim_cnt:��������ʱ��
//  ��    ע: ��ʱ�����õ���������ʱ��
//	��    ��: 2021-04-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint16_t hcsr04_read_data(void)
{
	uint16_t tim_cnt = 0;
	uint16_t time_start, time_end;
	
	hcsr04_start();
	
#if 1	//��ʱ����ʱ
	TIM_SetCounter(TIM4, 0);
	while (!ULT_ECHO_READ);	//�͵�ƽ�ȴ�
	
	time_start = TIM_GetCounter(TIM4);
	while (ULT_ECHO_READ)	//�ߵ�ƽ��ȡ����
	{
		;
	}
	time_end = TIM_GetCounter(TIM4);
	
	tim_cnt = time_end - time_start;
	
#else	
	while (!ULT_ECHO_READ);	//�͵�ƽ�ȴ�
	
	while (ULT_ECHO_READ)	//�ߵ�ƽ��ȡ����
	{
		tim_cnt++;	//����ʱ��??
		delay_us(1);
	}
	tim_cnt = tim_cnt * 2;
#endif	
	return tim_cnt;	//ʱ��us
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: hcsr04_measure
//	����˵��: ������ģ������������
//	��    ��: ��
//	�� �� ֵ: distance:�������
//  ��    ע: ��β���ȡƽ��ֵ
//	��    ��: 2021-04-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
float hcsr04_measure(void)
{
	uint8_t i;
	float distance = 0.0;
	uint16_t temp, sum_val = 0;
	
	for (i = 0; i < 5; i++)
	{
		temp = hcsr04_read_data();
//		printf("No : %d ; Value is %d \r\n", i, temp);
		sum_val += temp;
		delay_ms(60);
	}
	
	temp = sum_val / 5;	//5��ƽ��
//	printf("average value is %d \r\n", temp);
//	distance = ((temp / 1000000) * 340 * 100) / 2;	//�� cm Ϊ��λ
	distance = (float)temp / 58;
	
	return distance;
}


#endif	//__PRJ_STM32F40X_DRVULTRASONIC_C__
