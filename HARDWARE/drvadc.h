#ifndef __DRVADC_H__
#define __DRVADC_H__

#include "sys.h"

//GPIO 部分
#define RCC_PCLK_ADC_GPIO			RCC_AHB1Periph_GPIOC
#define PORT_ADC_IN					GPIOC
#define ADC_IN_PIN					GPIO_Pin_4

//ADC 部分
#define RCC_PCLK_ADC_CHL			RCC_APB2Periph_ADC1
#define ADC_CHL1					ADC1

//函数声明部分
extern void adc_init(void);
extern uint16_t get_adc_average(uint8_t adc_ch, uint8_t count);
extern void adc_test(void);

#endif	//__DRVADC_H__
