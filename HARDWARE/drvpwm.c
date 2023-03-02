//---------------------------------------------------------------------------------------------------------------------------------------------
//ƽ    ̨:				STM32F40X
//��    ��:    		 	drultrasonic.c
//��    ��:       		����AI
//��� ��:   			Vxxx
//�ļ��汾:   			V1.0.0
//��   ��:      		2021��05��29��
//˵   ��:      	 	������ģ�� HC-SR04 ����ʵ��
//----------------------------------------------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVPWM_C__
#define __PRJ_STM32F40X_DRVPWM_C__


#include "drvpwm.h"
//#include "delay.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: pwm_gpio_init
//	����˵��: PWM���GPIO����
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2021-05-30
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_gpio_init(void)
{
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(PWM1_GPIO_CLK_ALLENABLE, ENABLE);	 //ʹ��PA�˿�ʱ��
	GPIO_PinAFConfig(PWM1_GPIO_PORT, GPIO_PinSource1, GPIO_AF_TIM2); //GPIOA1����ΪTIM2_CH2
	
	gp_init.GPIO_Pin 	= PWM1_CON_IO;			//LPA.1 �˿�����
	gp_init.GPIO_Mode 	= GPIO_Mode_AF;			//�������ģʽ
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		//�������
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//IO���ٶ�Ϊ50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//����
	GPIO_Init(PWM1_GPIO_PORT, &gp_init);					 	 
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: pwm_timer_init
//	����˵��: PWM ��ʱ����������
//	��    ��: 	timer_no����ʱ�����
//				timer_arr���Զ���װֵ
//				timer_psc��ʱ�ӷ�Ƶϵ��
//	�� �� ֵ: ��
//	��    ��: 2020-05-30
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_timer_init(uint8_t timer_no, uint16_t timer_arr, uint16_t timer_psc)
{
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
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: pwm_configure
//	����˵��: PWM ��������
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-05-30
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_configure(void)
{
	TIM_OCInitTypeDef pwm_init_config;
	
	pwm_init_config.TIM_OCMode		= TIM_OCMode_PWM2;	//PWMͨ��2���
	pwm_init_config.TIM_OutputState	= TIM_OutputState_Enable;	//PWM���ʹ��
	pwm_init_config.TIM_OCPolarity	= TIM_OCPolarity_Low;	//�������Ϊ��
	TIM_OC2Init(TIM2, &pwm_init_config);
	
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);	//Ԥ����ʹ��
	TIM_ARRPreloadConfig(TIM2, ENABLE);	//ARPEʹ��
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: pwm_init
//	����˵��: PWM ��ʼ����������
//	��    ��: pwm_no���������
//	�� �� ֵ: ��
//	��    ��: 2020-05-30
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void pwm_init(uint8_t pwm_no)
{
	if (pwm_no == 1)
	{
		pwm_gpio_init();
		pwm_timer_init(2, 500-1, 84-1);
		pwm_configure();
		
		TIM_Cmd(TIM2, ENABLE);
	}
}

#endif	//__PRJ_STM32F40X_DRVPWM_C__
