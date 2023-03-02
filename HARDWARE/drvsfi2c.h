#ifndef __DRVSFI2C_H__
#define __DRVSFI2C_H__

#include "sys.h"

//#ifdef SOFT_I2C_COMM

#define RCC_PCLK_I2C_GPIO			RCC_AHB1Periph_GPIOB

#define PORT_I2C_SCL			GPIOB
#define PORT_I2C_SDA			GPIOB

#define I2C_SCL_IO					GPIO_Pin_0
#define I2C_SDA_IO					GPIO_Pin_1

#if 1
	/* �������룺 1 ѡ��GPIO�Ŀ⺯��ʵ��IO��д */
//#define I2C_SCL_LOW				GPIO_ResetBits(PORT_I2C_SCL, I2C_SCL_IO)	
//#define I2C_SCL_HIGH			GPIO_SetBits(PORT_I2C_SCL, I2C_SCL_IO)
//#define I2C_SDA_LOW				GPIO_ResetBits(PORT_I2C_SDA, I2C_SDA_IO)
//#define I2C_SDA_HIGH			GPIO_SetBits(PORT_I2C_SDA, I2C_SDA_IO)

#define I2C_SCL_0()				GPIO_ResetBits(PORT_I2C_SCL, I2C_SCL_IO)	
#define I2C_SCL_1()				GPIO_SetBits(PORT_I2C_SCL, I2C_SCL_IO)
#define I2C_SDA_0()				GPIO_ResetBits(PORT_I2C_SDA, I2C_SDA_IO)
#define I2C_SDA_1()				GPIO_SetBits(PORT_I2C_SDA, I2C_SDA_IO)

#define I2C_SDA_READ  GPIO_ReadInputDataBit(PORT_I2C_SDA, I2C_SDA_IO)
	
#else
	/* ѡ��ֱ�ӼĴ�������ʵ��IO��д */
#define I2C_SCL_LOW				(PORT_I2C_SCL->BRR  = I2C_SCL_IO)
#define I2C_SCL_HIGH			(PORT_I2C_SCL->BSRR = I2C_SCL_IO )
#define I2C_SDA_LOW				(PORT_I2C_SDA->BRR  = I2C_SDA_IO)
#define I2C_SDA_HIGH			(PORT_I2C_SDA->BSRR = I2C_SDA_IO)

#define I2C_SDA_READ			(PORT_I2C_SDA->IDR & I2C_SDA_IO)//��ȡ�����ƽ״̬,�ж�IO�ĵ�ƽ

#endif

#define I2C_ACK					0				//Ӧ��
#define I2C_NACK				1				//��Ӧ��

#define I2C_OK					0
#define I2C_ERR					1


typedef struct 
{
	uint16_t uiI2cSpeed;
}St_I2cInfo;

extern St_I2cInfo StI2cInfo;


extern void I2c_GpioInit(void);
extern void I2c_CommSpeedSet(uint16_t _usSpeed);
extern void I2c_Init(void);
extern void I2c_Delay(uint16_t _usTime);
extern void I2c_Start(void);
extern void I2c_Stop(void);
extern void I2c_SendOneByte(uint8_t _ucData);
extern uint8_t I2c_RecvOneByte(uint8_t _ucAck);
extern uint8_t I2c_WaitAck(uint16_t _usErrTime);
extern uint8_t I2c_WaitAckDelay(uint16_t _usErrTime);	//��ʱ�ȴ�Ӧ��
extern void I2c_GetAck(void);
extern void I2c_GetNack(void);

//#endif

#endif	//__DRVSFI2C_H__

