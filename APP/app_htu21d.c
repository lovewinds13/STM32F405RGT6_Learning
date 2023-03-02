 #ifndef __PRJ_STM32F10X_APP_HTU21D_C__
#define __PRJ_STM32F10X_APP_HTU21D_C__

#include <stdio.h>
#include <math.h>
#include "drvsfi2c.h"
#include "app_htu21d.h"
#include "delay.h"
#include "drvuart.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: Htu_Init
//	����˵��: ��������ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//  ��    ע: 
//	��    ��: 2020-03-11
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void Htu_Init(void)
{
	  
	I2c_Init();
	I2c_Start();
	I2c_SendOneByte(HTU_ADDR_WR);	//дI2C������ַ
//	I2c_WaitAck(200);
	I2c_WaitAckDelay(200);
	I2c_SendOneByte(HTU_SOFTWARE_RESET);		//��λ
//	I2c_WaitAck(200);
	I2c_WaitAckDelay(200);
	I2c_Stop();
	delay_ms(15);		//��λʱ�������Ҫ15ms
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: Htu_Measure
//	����˵��: Htu21d ��ʪ�ȶ�ȡ
//	��    ��: 	_ucOrder���¶� or ʪ�ȶ�ȡ����
//	�� �� ֵ: ��
//	��    ��: 2020-03-16
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------------------------
#if 1
float Htu_Measure(uint8_t _ucOrder)
{
	 uint8_t ucMsb = 0;
	 uint8_t ucLsb = 0;
	 volatile float fTemp = 0.0;
	 volatile float fHumi = 0.0;
	 volatile float fRetVal = 0.0;
	 
	 I2c_Start();
	 
	 I2c_SendOneByte(HTU_ADDR_WR);		//��ַ+д����
//	 if(I2c_WaitAck(200)==I2C_NACK)
	if(I2c_WaitAckDelay(200)==I2C_NACK)
	{
		 return 1;
	}
	 
	I2c_SendOneByte(_ucOrder);		//������������
//	if(I2c_WaitAck(200)==I2C_NACK)
	if(I2c_WaitAckDelay(200)==I2C_NACK)
	{
		 return 1;
	}
	
	delay_ms(50);	//14λ����ʱ�䷶Χ(44-58ms)
	 
	do{
	//	delay_ms(5);
		I2c_Start();	//��������
		I2c_SendOneByte(HTU_ADDR_RD);		//��ַ+������
//	}while(I2c_WaitAck(1)==I2C_NACK);
	}while(I2c_WaitAckDelay(1)==I2C_NACK);

	ucMsb = I2c_RecvOneByte(I2C_ACK);	//��������ACK
	ucLsb = I2c_RecvOneByte(I2C_NACK);	//��ȡ�����һ�ֽڷ���NACK

	I2c_Stop();

	ucLsb &= 0xFC;		//���÷ֱ���,�����λΪ0,�¶�:14λ;ʪ��:12λ 	
	fRetVal = ucMsb * 256 + ucLsb;/*MSB=(MSB<<=8)+LSB;����MSB��λ����8λ*/

	if(_ucOrder == HTU_TEMP)
	{
		fTemp = (175.72) * fRetVal / 65536 - 46.85;//�¶�:T= -46.85 + 175.72 * ST/2^16

		return fTemp;
	}
	else if(_ucOrder == HTU_HUMI)
	{
		fHumi = (fRetVal * 125) / 65536 - 6;//ʪ��: RH%= -6 + 125 * SRH/2^16

		return fHumi;
	}
	else
	{
		return 1;
	}
} 

#else

float Htu_Measure(uint8_t _ucOrder)
{
	uint8_t ucaRecvBuf[2] = {0};
	uint8_t ucTmpVal = _ucOrder;
	volatile float fTemp = 0.0;
	volatile float fHumi = 0.0;
	volatile float fRetVal = 0.0;
	 
	htu_write_some_bytes(&ucTmpVal, 1);		//д���������

	delay_ms(50);	//14λ����ʱ�䷶Χ(44-58ms)
	
	htu_read_some_bytes(ucaRecvBuf, 2);	//�������ֽ�����
			 
	ucaRecvBuf[1] &= 0xFC;		//���÷ֱ���,�����λΪ0,�¶�:14λ;ʪ��:12λ 	
	fRetVal = (ucaRecvBuf[0] << 8) | ucaRecvBuf[1];	// MSB=(MSB<<=8)+LSB;����MSB��λ����8λ

	if(_ucOrder == HTU_TEMP)
	{
		 fTemp = (175.72) * fRetVal / 65536 - 46.85;//�¶�:T= -46.85 + 175.72 * ST/2^16
		 
		 return fTemp;
	}
	else if(_ucOrder == HTU_HUMI)
	{
		 fHumi = (fRetVal * 125) / 65536 - 6.00;//ʪ��: RH%= -6 + 125 * SRH/2^16

		 return fHumi;
	}
	else
	{
		return 1;
	}
} 
#endif

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: htu_write_some_bytes
//	����˵��: htu21d ͨ��ICд����ֽ�����
//	��    ��: 	pbdata��д�������
//				write_length��д�����ݵĳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-19
//  ��    ע: ����I2C���Ͷ��ֽ�����ʱ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t htu_write_some_bytes(uint8_t *pbdata, uint16_t write_length)
{
	I2c_Start();

	I2c_SendOneByte(HTU_ADDR_WR);
	if (I2C_NACK == I2c_WaitAck(200))
	{
		return 1;
	}
	
	//forѭ�����Ͷ���ֽ�����
	for (uint16_t i = 0; i < write_length; i++)
	{
		I2c_SendOneByte(pbdata[i]);
		if (I2C_NACK == I2c_WaitAck(200))
		{
			return 1;
		}
	}

	//whileѭ�����Ͷ���ֽ�����
//	while (write_length--)
//	{
//		I2c_SendOneByte(*pbdata++);
//		if (I2C_NACK == I2c_WaitAck(200))
//		{
//			return false;
//		}
//	}

//	I2c_Stop();

	return 0;
	
}

//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: htu_read_some_bytes
//	����˵��: htu21d ͨ��IC��ȡ���ֽ�����
//	��    ��: 	pbdata��д�������
//				read_length��д�����ݵĳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-19
//  ��    ע: ����I2C���ն��ֽ�����ʱ��
//	��    ��: by ����AI
//-------------------------------------------------------------------------------------------------------------------------
uint8_t htu_read_some_bytes(uint8_t *pbdata, uint16_t read_length)
{
	I2c_Start();

	I2c_SendOneByte(HTU_ADDR_RD);
	if (I2C_NACK == I2c_WaitAck(200))
	{
		return 1;
	}

	for (uint16_t i = 0; i < read_length - 1; i++)
	{
		*pbdata++ = I2c_RecvOneByte(I2C_ACK);
	}
	*pbdata++ = I2c_RecvOneByte(I2C_NACK);	//�������һ���ֽڷ���NACK,���ߴӻ��������Ѿ����

	I2c_Stop();

	return 0;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Htu_Display
//	����˵��: ����������ʾ
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-16
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
 void Htu_Display(void)
{
	u16 usTemp = 0;
	volatile double f_RetVal = 0.0;
	u8 ucTest[10] = {0};
	
	f_RetVal = Htu_Measure(HTU_TEMP);//�õ��¶�ֵ
	printf("The htu measure temp is :%4.2fC \r\n", f_RetVal);
	
	sprintf((char*)ucTest,"%4.2f", f_RetVal);		//LCD��ʾ��ʽ1��sprintf�����������ӡ��test������,ת�����ַ���
	printf("test is %sC \r\n", ucTest);
	printf("\r\n");

	
	usTemp = f_RetVal;			//LCD��ʾ��ʽ2:���õ�����ֵ��ֳ�������С��ֱ����ʾ��Һ��
	f_RetVal -= usTemp;
	f_RetVal *= 100;		//������λС��

		
	f_RetVal= Htu_Measure(HTU_HUMI);		//�õ�ʪ��ֵ
	printf("The htu measure humi is :%4.2fRH \r\n", f_RetVal);
	usTemp = f_RetVal;
	f_RetVal -= usTemp;
	f_RetVal *= 100;
	printf("\r\n");
	
}

#endif	//__PRJ_STM32F10X_APP_HTU21D_C__
