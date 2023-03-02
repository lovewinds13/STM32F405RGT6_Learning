#ifndef __DRVULTRASONIC_H__
#define __DRVULTRASONIC_H__

#define RCC_PCLK_ULT_GPIO			RCC_AHB1Periph_GPIOB
#define PORT_ULT_GPIO				GPIOB

#define ULT_TRIG_PIN	GPIO_Pin_0
#define	ULT_ECHO_PIN	GPIO_Pin_1

#define ULT_ECHO_READ		GPIO_ReadInputDataBit(PORT_ULT_GPIO, ULT_ECHO_PIN)

extern void hcsr04_gpio_init(void);
extern float hcsr04_measure(void);

#endif	//__DRVULTRASONIC_H__

