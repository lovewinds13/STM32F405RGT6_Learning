#ifndef _DRVPWM_H_
#define _DRVPWM_H_ 

#include "sys.h"


#define PWM1_CON_IO		GPIO_Pin_1
//#define LED1_CON_IO		GPIO_Pin_2
#define PWM1_GPIO_PORT	GPIOA
//#define LED1_GPIO_PORT	GPIOA

#define PWM1_GPIO_CLK_ALLENABLE  (RCC_AHB1Periph_GPIOA)

extern void pwm_init(uint8_t pwm_no);

#endif	//_DRVPWM_H_
