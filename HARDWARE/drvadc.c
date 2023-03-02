#ifndef __PRJ_STM32F40X_DRVADC_C__
#define __PRJ_STM32F40X_DRVADC_C__

//---------------------------------------------------------------------------------------------------------------------------------------------
//平    台:				STM32F40X
//文    件:    		 	drvadc.c
//作    者:       		霁风AI
//库版 本:   			Vxxx
//文件版本:   			V1.0.0
//日   期:      		2021年06月03日
//说   明:      	 	ADC驱动实现
//----------------------------------------------------------------------------------------------------------------------------------------------

#include "drvadc.h"
#include "drvoled.h"
#include "delay.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: adc_gpio_init
//	功能说明: ADC输入采集引脚配置
//	形    参: 无
//	返 回 值: 无
//	日    期: 2020-03-11
//  备    注: 
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_gpio_init(void)
{	
	GPIO_InitTypeDef gpio_initx;
	
	RCC_AHB1PeriphClockCmd(RCC_PCLK_ADC_GPIO, ENABLE);	 //使能PC端口时钟
	
	//PC4 作为模拟通道输入引脚                         
	gpio_initx.GPIO_Pin 	= ADC_IN_PIN;	//ADC输入引脚
	gpio_initx.GPIO_Mode 	= GPIO_Mode_AN;		//模拟输入引脚
	gpio_initx.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	
	GPIO_Init(PORT_ADC_IN, &gpio_initx);	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: Adc_Config
//	功能说明: ADC输入采集功能配置
//	形    参: 无
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-06-03
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_config(void)
{ 	
	ADC_InitTypeDef adc_init_config; 
	ADC_CommonInitTypeDef adc_comm_config;
	
	RCC_APB2PeriphClockCmd(RCC_PCLK_ADC_CHL, ENABLE );	  //使能ADC1通道时钟

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE); //ADC1 复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE); 	//复位结束

	adc_comm_config.ADC_Mode 					= ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	adc_comm_config.ADC_TwoSamplingDelay 		= ADC_TwoSamplingDelay_5Cycles;	//两个采样阶段之间的延迟 5 个时钟
	adc_comm_config.ADC_DMAAccessMode 			= ADC_DMAAccessMode_Disabled; //DMA 失能
	adc_comm_config.ADC_Prescaler 				= ADC_Prescaler_Div4;	//预分频 4 分频
	ADC_CommonInit(&adc_comm_config);	

	adc_init_config.ADC_Resolution 				= ADC_Resolution_12b;//12 位模式
	adc_init_config.ADC_ScanConvMode 			= DISABLE;	//非扫描模式
	adc_init_config.ADC_ContinuousConvMode 		= DISABLE;	//关闭连续转换
	adc_init_config.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	adc_init_config.ADC_DataAlign 				= ADC_DataAlign_Right;	//右对齐
	adc_init_config.ADC_NbrOfConversion 		= 1;	//1 个转换在规则序列中
	ADC_Init(ADC_CHL1, &adc_init_config);//ADC1 初始化
	
	ADC_Cmd(ADC_CHL1, ENABLE);	//开启 ADC1 转换器
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: Adc_Init
//	功能说明: ADC初始化
//	形    参: 无
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-06-03
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_init(void)
{
	adc_gpio_init();	//IO初始化
	adc_config();	//ADC功能配置
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: get_adc_convert_val
//	功能说明: 输出ADC转换结果
//	形    参: 	adc_ch：ADC 采集通道
//	返 回 值: 采集数据结果
//  备    注: 
//	日    期: 2020-06-03
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint16_t get_adc_convert_val(uint8_t adc_ch)   
{
	ADC_RegularChannelConfig(ADC_CHL1, adc_ch, 1, ADC_SampleTime_480Cycles );	//设置指定 ADC 的规则组通道，一个序列，采样时间
	ADC_SoftwareStartConv(ADC_CHL1); //使能指定的 ADC1 的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC_CHL1, ADC_FLAG_EOC));//等待转换结束
	
	return ADC_GetConversionValue(ADC_CHL1); //返回最近一次 ADC1 规则组的转换结果
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: get_adc_average
//	功能说明: 转换出 ADC 采集数据
//	形    参: 	adc_ch：ADC 通道号
//				count：采集次数
//	返 回 值: 转换数据
//  备    注: 
//	日    期: 2020-03-11
//	作    者: by 霁风AI
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
//	函 数 名: adc_test
//	功能说明: ADC单通道采样测试
//	形    参: 无
//	返 回 值: 无
//  备    注: 
//	日    期: 2020-03-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void adc_test(void)
{
	uint16_t adc_val = 0;
	float tmp_val = 0.0;
	
	adc_val = get_adc_average(ADC_Channel_14, 10);	//ADC1 14通道
	oled_dis_num(2+64, 2, adc_val, 4, 16, 0x80);//显示ADC的值
	
	//INPUT VOLTAGE = (ADC Value / ADC Resolution) * Reference Voltage
	tmp_val = (float)adc_val * (3.3 / 4096);	//12位ADC
//	printf("voltage is %04fv.\r\n", tmp_val);
	
	adc_val = tmp_val;	//整数部分
//	printf("Voltage integer of usAdcVal is :%d \r\n", usAdcVal);
	oled_dis_num(2+64, 4, adc_val, 1, 16, 0x80);//显示电压值
	
	tmp_val -= adc_val;	//小数部分
//	printf("Voltage float of is :%f \r\n",fTmpVal);
	tmp_val *= 1000;
	oled_dis_num(2+64+16, 4, tmp_val, 3, 16, 0x80);
	
	delay_ms(250);	
}

#endif /* __PRJ_STM32F40X_DRVADC_C__ */
