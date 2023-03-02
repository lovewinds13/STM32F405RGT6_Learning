#ifndef _DRVBUTTON_H_
#define _DRVBUTTON_H_ 

//#include "Typedef.h"
//#include "stm32f10x.h"
#include "sys.h"

//GPIO 部分
#define RCC_BUTTON_GPIO_CLK			RCC_AHB1Periph_GPIOB

//PB6/PB7 默认是I2C1
#define BUTTON1_IO					GPIO_Pin_7
#define BUTTON2_IO					GPIO_Pin_6

#define BUTTONS_PORT				GPIOB
#define BUTTON1_PORT				GPIOB
#define BUTTON2_PORT				GPIOB

#define KEY1_VAL	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)
#define KEY2_VAL	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)

#define KEY1_DOWN	0x01
#define KEY2_DOWN	0x02

extern void button_gpio_init(void);
extern uint8_t button_detect(uint8_t key_mode);

#endif	//_DRVBUTTON_H_
