//-----------------------------------------------------------------------------------------------------------
//
//	ģ������ : BSPģ��(For STM32F405RGT6)
//	�ļ����� : bsp.c
//	��    �� : V1.0
//	˵    �� : ����Ӳ���ײ�������������ļ���ÿ��c�ļ����� #include "bsp.h" ���������е���������ģ�顣
//			   bsp = Borad surport packet �弶֧�ְ�
//	�޸ļ�¼ :
//		�汾��  ����         ����       ˵��
//		V1.0    2021-05-25  ����AI   ��ʽ����
//
//	Copyright (C), 2021-2030, ΢�Ź��ں� TECHTIMES
//
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//	STM32F405RGT6������LED���߷��䣺
//		LED0     : PA1          (�͵�ƽ�������ߵ�ƽϨ��)
//		LED1     : PA2          (�͵�ƽ�������ߵ�ƽϨ��)
//------------------------------------------------------------------------------------------------------------

#include "drvled.h"
#include "delay.h"

//#define DRIVER_LED_REG	//�Ĵ�����ʽѡ����ƿ���

//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void Bsp_LedInit(void)
{
#ifdef DRIVER_LED_REG
	RCC->APB2ENR	|= 1<<2;	//GPIOA ʱ��ʹ��
	RCC->APB2ENR	|= 1<<5;	//GPIOD ʱ��ʹ��
	
	GPIOA->CRH		&= 0xfffffff0;	
	GPIOA->CRH		|= 0x00000003;	//�������,IO���ٶ�Ϊ50MHz
	GPIOA->ODR		|= 1<<8;	//PA8 ����ߵ�ƽ
	
	GPIOD->CRL		&= 0xfffff0ff;
	GPIOD->CRL		|= 0x00000300;	//�������,IO���ٶ�Ϊ50MHz
	GPIOD->ODR		|= 1<<2;	//PD2 ����ߵ�ƽ
#else
	GPIO_InitTypeDef  gp_init;

	RCC_AHB1PeriphClockCmd(LED_GPIO_CLK_ALLENABLE, ENABLE);	 //ʹ��PA�˿�ʱ��

	gp_init.GPIO_Pin 	= LED0_CON_IO;				 //LED0-->PA.1 �˿�����
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//��ͨ���ģʽ
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //�������
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//����
	GPIO_Init(LED0_GPIO_PORT, &gp_init);					 	

	gp_init.GPIO_Pin 	= LED1_CON_IO;	    		 //LED1-->PA.2 �˿�����, �������
	gp_init.GPIO_Mode 	= GPIO_Mode_OUT;			//��ͨ���ģʽ
	gp_init.GPIO_OType 	= GPIO_OType_PP; 		 //�������
	gp_init.GPIO_Speed 	= GPIO_Speed_50MHz;		//������� ��IO���ٶ�Ϊ50MHz
	gp_init.GPIO_PuPd 	= GPIO_PuPd_UP;			//����
	GPIO_Init(LED1_GPIO_PORT, &gp_init);	  				
	
	GPIO_SetBits(LED0_GPIO_PORT, LED0_CON_IO);
	GPIO_SetBits(LED1_GPIO_PORT, LED1_CON_IO); 		
#endif	
}

//-------------------------------------------------------------------------------------------------------
//	�� �� ��: bsp_LedOn
//	����˵��: ����ָ����LEDָʾ��,���������
//	��    ��:  _no : ָʾ����ţ���Χ 0 - 1
//	�� �� ֵ: ��
//-------------------------------------------------------------------------------------------------------
void Bsp_LedOn(uint8_t _no)
{
#ifdef DRIVER_LED_REG
	if (_no == 0)
	{
		 GPIOA->ODR |= 0x0100;//1<<8;
	}
	else if (_no == 1)
	{
		 GPIOD->ODR |= 1<<2;
	}
#else	
	if (_no == 0)
	{
		 GPIO_ResetBits(LED0_GPIO_PORT, LED0_CON_IO);
	}
	else if (_no == 1)
	{
		 GPIO_ResetBits(LED1_GPIO_PORT, LED1_CON_IO);
	}
#endif
}

//-------------------------------------------------------------------------------------------------------
//	�� �� ��: Bsp_LedOff
//	����˵��: �ر�ָ����LEDָʾ��
//	��    ��:  _no : ָʾ����ţ���Χ 0 - 1
//	�� �� ֵ: ��
//-------------------------------------------------------------------------------------------------------
void Bsp_LedOff(uint8_t _no)
{	
#ifdef DRIVER_LED_REG
	if (_no == 0)
	{
		 GPIOA->ODR &= 0xfeff;//0<<8;
	}
	else if (_no == 1)
	{
//		 GPIOD->ODR &= 0<<2;
		GPIOD->ODR	&= ~(1<<2);
	}
#else
	if (_no == 0)
	{
		 GPIO_SetBits(LED0_GPIO_PORT, LED0_CON_IO);
	}
	else if (_no == 1)
	{
		 GPIO_SetBits(LED1_GPIO_PORT, LED1_CON_IO);
	}
#endif
}

//-------------------------------------------------------------------------------------------------------
//	�� �� ��: bsp_LedToggle
//	����˵��: ��תָ����LEDָʾ�ơ�
//	��    ��:  _no : ָʾ����ţ���Χ 0 - 1
//	�� �� ֵ: ��
//-------------------------------------------------------------------------------------------------------
void Bsp_LedToggle(uint8_t _no)
{
	uint8_t flag = _no % 2;
	
	if (flag == 0)
	{
		LED0_GPIO_PORT->ODR ^= LED0_CON_IO;
	}
	else if (flag == 1)
	{
		LED1_GPIO_PORT->ODR ^= LED1_CON_IO;
	}
}

//-------------------------------------------------------------------------------------------------------
//	�� �� ��: Bsp_LedTest
//	����˵��: LED���������ԡ�
//	��    ��:  _uiTime : ��ʱʱ��
//	�� �� ֵ: ��
//-------------------------------------------------------------------------------------------------------
void Bsp_LedTest(uint16_t _uiTime)
{
	Bsp_LedToggle(0);
	delay_ms(_uiTime);
	Bsp_LedToggle(1);
	delay_ms(_uiTime);
}


//***************************** ΢�Ź��ں� TECHTIMES (END OF FILE) *********************************/
