//--------------------------------------------------------------------------------------------------------
//
//	ģ������ : ���ģ��I2Cʱ��
//	�ļ����� : drvsfi2c.c
//	��    �� : V1.0
//	˵    �� : 
//				(1) 
//				(2) 
//				(3) 
//				(4) 
//
//	�޸ļ�¼ :
//		�汾��     ����        ����     ˵��
//		V1.0    2019-12-29  ����AI  ��ʽ����
//		V1.1    
//		V1.2	
//		V1.3	
//
//	Copyright (C), 2020-2030, ΢�Ź��ںš���TECHTIMES
//
//--------------------------------------------------------------------------------------------------------


#ifndef __PRJ_STM32F40X_DRVSFI2C_C__
#define __PRJ_STM32F40X_DRVSFI2C_C__

#include "drvsfi2c.h"//оƬGPIO����
#include "delay.h"

#define SOFT_I2C_COMM

#ifdef SOFT_I2C_COMM

#define IO_OD_MODE 0	//IIC ��SDA/SCL��IO����Ϊ��©ģʽ(��©��ʱ����Ҫ�л�IO����)

St_I2cInfo StI2cInfo;

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_GpioInit
//	����˵��: ���ģ��I2C IO��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_GpioInit(void)
{
	GPIO_InitTypeDef gpio_config_init;
	
	RCC_AHB1PeriphClockCmd(RCC_PCLK_I2C_GPIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//����SWD��ʧ��JTAG
	
	gpio_config_init.GPIO_Pin 	= I2C_SCL_IO | I2C_SDA_IO;
	gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
#if IO_OD_MODE
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_config_init.GPIO_OType = GPIO_OType_OD;	//��©���
//	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
#else
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_config_init.GPIO_OType = GPIO_OType_PP;	//�������
	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;  
#endif
	GPIO_Init(PORT_I2C_SCL, &gpio_config_init);
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2C_SDASetInput
//	����˵��: I2C��ȡ���ݷ�������Ϊ����(�����©,�����л�)
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2C_SDASetInput(void)
{
#if IO_OD_MODE
	
#else	//IO�ǿ�©��Ҫ�л�����
	GPIO_InitTypeDef gpio_config_init;
	
	gpio_config_init.GPIO_Pin 	= I2C_SDA_IO;
	gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_IN;//��������	
	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_NOPULL;
//	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
//	gpio_config_init.GPIO_OType = GPIO_OType_PP;	//�������
//	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(PORT_I2C_SDA, &gpio_config_init);
#endif
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2C_SDASetOutput
//	����˵��: I2C��ȡ���ݷ�������Ϊ���
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2C_SDASetOutput(void)
{
#if IO_OD_MODE
	
#else
	GPIO_InitTypeDef gpio_config_init;
	
	gpio_config_init.GPIO_Pin = I2C_SDA_IO;
	gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_config_init.GPIO_OType = GPIO_OType_PP;	//�������
	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;	
	GPIO_Init(PORT_I2C_SDA, &gpio_config_init);
#endif
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_CommSpeedSet
//	����˵��: I2Cͨ���ٶȿ���
//	��    ��: _uiSpeed:��ʱʱ��(us)
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_CommSpeedSet(uint16_t _usSpeed)
{
	StI2cInfo.uiI2cSpeed = _usSpeed;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Init
//	����˵��: I2C��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-15
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_Init(void)
{
	I2c_GpioInit();
	I2c_CommSpeedSet(2);
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Delay
//	����˵��: I2C��ʱ
//	��    ��: _usTime:��ʱʱ��(us)
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_Delay(uint16_t _usTime)
{
	delay_us(_usTime);
}
	
//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Start
//	����˵��: I2C��ʼ�ź�
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_Start(void)
{
	I2C_SDASetOutput();
	I2C_SDA_1();						//����SDA��
	I2C_SCL_1();						//����SCL��
	I2c_Delay(StI2cInfo.uiI2cSpeed);					//��ʱ���ٶȿ���
	
	I2C_SDA_0();						//��SCL��Ϊ��ʱ��SDA��һ���½��ش���ʼ�ź�
	I2c_Delay(StI2cInfo.uiI2cSpeed);					//��ʱ���ٶȿ���
	I2C_SCL_0();	
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Stop
//	����˵��: I2Cֹͣ�ź�
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_Stop(void)
{
	I2C_SDASetOutput();
	I2C_SDA_0();
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	
	I2C_SCL_1();
	I2C_SDA_1();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_SendOneByte
//	����˵��: I2C����һ���ֽ�����
//	��    ��: 
//				_ucData:���͵�һ�ֽ�����
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_SendOneByte(uint8_t _ucData)
{
	uint8_t ucCnt;
	
	I2C_SDASetOutput();	//SDA����Ϊ���(��IOΪ��©,������з����л�)
	for(ucCnt = 0; ucCnt < 8; ucCnt++)
	{
		I2C_SCL_0();		//SCL�͵�ƽ,�������ݸı�
		I2c_Delay(StI2cInfo.uiI2cSpeed);
		
		if(_ucData & 0x80)		//�Ӹ�λ��ʼ����
		{
			I2C_SDA_1();		
		}
		else
		{
			I2C_SDA_0();		
		}
		
		_ucData <<= 1;
		I2c_Delay(StI2cInfo.uiI2cSpeed);
		
		I2C_SCL_1();		//�����ȶ�,���͸��ӻ�
		I2c_Delay(StI2cInfo.uiI2cSpeed);
	}
	I2C_SCL_0();		//��9��ʱ��,SCL�͵�ƽ,�ȴ�Ӧ���ź�����
	I2c_Delay(StI2cInfo.uiI2cSpeed);
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_RecvOneByte
//	����˵��: I2C����һ���ֽ�����
//	��    ��: 
//				_ucAck:Ӧ���ж�(0:����Ӧ��;1:������Ӧ��)
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t I2c_RecvOneByte(uint8_t _ucAck)
{
	uint8_t ucCnt = 0, ucData = 0;
	
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	
	I2C_SDA_1();
	
	I2C_SDASetInput();		//SDA��©ʱ�������÷���,OD ��˫���IO 
	
	for(ucCnt = 0; ucCnt < 8; ucCnt++)
	{
		I2C_SCL_1();		//SCL�ߵ�ƽʱSDA�ϵ����ݴﵽ�ȶ�
		I2c_Delay(StI2cInfo.uiI2cSpeed);		//��ʱ�ȴ��ź��ȶ�
		
		ucData <<= 1;
		if(I2C_SDA_READ)
		{
			ucData |= 0x01;
		}
		else
		{
			ucData &= 0xfe;		
		}
		I2C_SCL_0();		//�������ݸı�
		I2c_Delay(StI2cInfo.uiI2cSpeed);
	}
	I2C_SDASetOutput();	//SDA��©ʱ�������÷���,OD ��˫���IO 
	if(_ucAck)
	{
		I2c_GetNack();
	}
	else
	{
		I2c_GetAck();
	}
	
	return ucData;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Wait_Ack
//	����˵��: I2C�ȴ�Ӧ��
//	��    ��: 
//				_usErrTime:��ʱʱ������
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//  ��    ע��ע��I2c_Stop(),�˺���ʵ����#else������
//				Ӧ��ʱ������ʱ�ȴ�,��������ԭ��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
#if 1
uint8_t I2c_WaitAckDelay(uint16_t _usErrTime)
{	
	uint16_t temp = _usErrTime;
	
	I2C_SDASetInput();      //SDA����Ϊ����,��©ʱ�������÷���,OD ��˫���IO  

	I2C_SDA_1();
	I2c_Delay(StI2cInfo.uiI2cSpeed);	   
	I2C_SCL_1();
	I2c_Delay(StI2cInfo.uiI2cSpeed);	 
	while (I2C_SDA_READ)
	{
		temp--;
		if (temp == 0)
		{
			I2c_Stop();
			return I2C_NACK;
		}
	}
	I2C_SCL_0();//ʱ������,�����շ�����  
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	
	return I2C_ACK;  
} 

//#else

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_Wait_Ack
//	����˵��: I2C�ȴ�Ӧ��
//	��    ��: 
//				_usErrTime:��ʱʱ������(�˺���������,����Ϊ�˺�#if���湫������)
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//  ��    ע��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t I2c_WaitAck(uint16_t _usErrTime)
{
	uint8_t ucAck = 0xFF;
	
	I2C_SDA_1();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SCL_1();		//��ʱ�ж��Ƿ���Ӧ��
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	
//	if(I2C_SDA_READ)
	while (I2C_SDA_READ && (_usErrTime--))	//����һ��ʱ���ȴ�,ȷ��Ӧ�����Ӧ���źŵ��ȶ�
	{
		ucAck = I2C_NACK;	
	}
//	else
	{
		ucAck = I2C_ACK;	
	}
	
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	
	return ucAck;
}
#endif

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_GetAck
//	����˵��: I2C�õ�Ӧ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_GetAck(void)
{
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SDASetOutput();
	I2C_SDA_0();	//�ھŸ�ʱ��,SDAΪ��Ӧ��
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SCL_1();		//SCL�ߵ�ƽ,�ߵ�ƽʱ��ȡSDA������
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SDA_1();		//�ͷ�SDA
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: I2c_GetNack
//	����˵��: I2C�õ���Ӧ��()
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2019-12-29
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void I2c_GetNack(void)
{
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SDASetOutput();
	I2C_SDA_1();	//�ھŸ�ʱ��,SDAΪ�߷�Ӧ��
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SCL_1();		//SCCL�ߵ�ƽ,�ߵ�ƽʱ��ȡSDA������
	I2c_Delay(StI2cInfo.uiI2cSpeed);
	I2C_SCL_0();
	I2c_Delay(StI2cInfo.uiI2cSpeed);
}

#endif	//SOFT_I2C_COMM

#endif /* __PRJ_STM32F10X_DRVSFI2C_C__ */
