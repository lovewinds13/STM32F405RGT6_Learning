#ifndef __PRJ_STM32F40X_DRVADC_C__
#define __PRJ_STM32F40X_DRVADC_C__

//---------------------------------------------------------------------------------------------------------------------------------------------
//ƽ    ̨:				STM32F40X
//��    ��:    		 	drvadc.c
//��    ��:       		����AI
//��� ��:   			Vxxx
//�ļ��汾:   			V1.0.0
//��   ��:      		2021��06��03��
//˵   ��:      	 	ADC����ʵ��
//----------------------------------------------------------------------------------------------------------------------------------------------

#include "drvadc.h"
#include "drvoled.h"
#include "delay.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: adc_gpio_init
//	����˵��: ADC����ɼ���������
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-11
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_gpio_init(void)
{	
	GPIO_InitTypeDef gpio_initx;
	
	RCC_AHB1PeriphClockCmd(RCC_PCLK_ADC_GPIO, ENABLE);	 //ʹ��PC�˿�ʱ��
	
	//PC4 ��Ϊģ��ͨ����������                         
	gpio_initx.GPIO_Pin 	= ADC_IN_PIN;	//ADC��������
	gpio_initx.GPIO_Mode 	= GPIO_Mode_AN;		//ģ����������
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	
	GPIO_Init(PORT_ADC_IN, &gpio_initx);	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: Adc_Config
//	����˵��: ADC����ɼ���������
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2021-06-03
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_config(void)
{ 	
	ADC_InitTypeDef adc_init_config; 
	ADC_CommonInitTypeDef adc_comm_config;
	
	RCC_APB2PeriphClockCmd(RCC_PCLK_ADC_CHL, ENABLE );	  //ʹ��ADC1ͨ��ʱ��

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE); //ADC1 ��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE); 	//��λ����

	adc_comm_config.ADC_Mode 					= ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	adc_comm_config.ADC_TwoSamplingDelay 		= ADC_TwoSamplingDelay_5Cycles;	//���������׶�֮����ӳ� 5 ��ʱ��
	adc_comm_config.ADC_DMAAccessMode 			= ADC_DMAAccessMode_Disabled; //DMA ʧ��
	adc_comm_config.ADC_Prescaler 				= ADC_Prescaler_Div4;	//Ԥ��Ƶ 4 ��Ƶ
	ADC_CommonInit(&adc_comm_config);	

	adc_init_config.ADC_Resolution 				= ADC_Resolution_12b;//12 λģʽ
	adc_init_config.ADC_ScanConvMode 			= DISABLE;	//��ɨ��ģʽ
	adc_init_config.ADC_ContinuousConvMode 		= DISABLE;	//�ر�����ת��
	adc_init_config.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	adc_init_config.ADC_DataAlign 				= ADC_DataAlign_Right;	//�Ҷ���
	adc_init_config.ADC_NbrOfConversion 		= 1;	//1 ��ת���ڹ���������
	ADC_Init(ADC_CHL1, &adc_init_config);//ADC1 ��ʼ��
	
	ADC_Cmd(ADC_CHL1, ENABLE);	//���� ADC1 ת����
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: Adc_Init
//	����˵��: ADC��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2021-06-03
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_init(void)
{
	adc_gpio_init();	//IO��ʼ��
	adc_config();	//ADC��������
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: get_adc_convert_val
//	����˵��: ���ADCת�����
//	��    ��: 	adc_ch��ADC �ɼ�ͨ��
//	�� �� ֵ: �ɼ����ݽ��
//  ��    ע: 
//	��    ��: 2020-06-03
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint16_t get_adc_convert_val(uint8_t adc_ch)   
{
	ADC_RegularChannelConfig(ADC_CHL1, adc_ch, 1, ADC_SampleTime_480Cycles );	//����ָ�� ADC �Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_SoftwareStartConv(ADC_CHL1); //ʹ��ָ���� ADC1 �����ת����������
	while(!ADC_GetFlagStatus(ADC_CHL1, ADC_FLAG_EOC));//�ȴ�ת������
	
	return ADC_GetConversionValue(ADC_CHL1); //�������һ�� ADC1 �������ת�����
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: get_adc_average
//	����˵��: ת���� ADC �ɼ�����
//	��    ��: 	adc_ch��ADC ͨ����
//				count���ɼ�����
//	�� �� ֵ: ת������
//  ��    ע: 
//	��    ��: 2020-03-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint16_t get_adc_average(uint8_t adc_ch, uint8_t count)
{
	uint32_t tmp_val;
	uint8_t i;
	
	for(i=0; i<count; i++)
	{
		tmp_val += get_adc_convert_val(adc_ch);
		delay_ms(5);
	}
	
	return tmp_val / count;
} 	

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: adc_test
//	����˵��: ADC��ͨ����������
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2020-03-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_test(void)
{
	uint16_t adc_val = 0;
	float tmp_val = 0.0;
	
	adc_val = get_adc_average(ADC_Channel_14, 10);	//ADC1 14ͨ��
	oled_dis_num(2+64, 2, adc_val, 4, 16, 0x80);//��ʾADC��ֵ
	
	//INPUT VOLTAGE = (ADC Value / ADC Resolution) * Reference Voltage
	tmp_val = (float)adc_val * (3.3 / 4096);	//12λADC
//	printf("voltage is %04fv.\r\n", tmp_val);
	
	adc_val = tmp_val;	//��������
//	printf("Voltage integer of usAdcVal is :%d \r\n", usAdcVal);
	oled_dis_num(2+64, 4, adc_val, 1, 16, 0x80);//��ʾ��ѹֵ
	
	tmp_val -= adc_val;	//С������
//	printf("Voltage float of is :%f \r\n",fTmpVal);
	tmp_val *= 1000;
	oled_dis_num(2+64+16, 4, tmp_val, 3, 16, 0x80);
	
	delay_ms(250);	
}

#endif /* __PRJ_STM32F40X_DRVADC_C__ */
