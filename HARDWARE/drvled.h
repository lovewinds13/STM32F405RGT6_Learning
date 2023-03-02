#ifndef _DRVLED_H_
#define _DRVLED_H_ 

//#include "Typedef.h"
//#include "stm32f10x.h"
#include "sys.h"

#define LED0 PAout(1)	// PA1
#define LED1 PAout(2)	// PA2	

#define LED0_CON_IO		GPIO_Pin_1
#define LED1_CON_IO		GPIO_Pin_2
#define LED0_GPIO_PORT	GPIOA
#define LED1_GPIO_PORT	GPIOA

#define LED_GPIO_CLK_ALLENABLE  (RCC_AHB1Periph_GPIOA)

extern void Bsp_LedInit(void);//≥ı ºªØ
extern void Bsp_LedOn(uint8_t _no);
extern void Bsp_LedOff(uint8_t _no);
extern void Bsp_LedToggle(uint8_t _no);
extern void Bsp_LedTest(uint16_t _uiTime);

#endif	//_DRVLED_H_
