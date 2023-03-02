//--------------------------------------------------------------------------------------------------------
//
//	ģ������ : ���Flash��д����
//	�ļ����� : drvexflash.c
//	��    �� : V1.0
//	˵    �� : 
//				(1) drvexflash.c����оƬSpiͨ�Žӿ�
//				(2) drvexflash.c�����W25X����Flash�Ķ�д����
//				(3) 
//				(4) 
//
//	�޸ļ�¼ :
//		�汾��     ����        ����     ˵��
//		V1.0    2020-03-07  ����AI  ��ʽ����
//		V1.1    
//		V1.2	
//		V1.3	
//
//	Copyright (C), 2020-2030, ΢�Ź��ںš���TECHTIMES
//
//--------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------------
//˵��:SPI����Flash(25Q16)
//W25X16оƬ������2MB (16Mbit)
//          ҳ����16*16*32 (2M/256)
//          ��������16*32
//          ������32

//2����д������
//�� ------------ һ������һҳ(256Bytes)
//д ------------ ҳ
//���� ---------- �������顢����оƬ

//3�����ƺ�״̬�Ĵ�������(Ĭ��:0x00)
//BITλ  7   6   5   4   3   2   1   0
//      SPR  RV  TB  BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1--æ;0--����)
//--------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DEVEXFLASH_C__
#define __PRJ_STM32F40X_DEVEXFLASH_C__

#include "hardware_spi.h"
#include "drvexflash.h"
//#include "drvsfspi.h"
#include "delay.h"

#define SPI_COMM_MODE	1	//ȷ����Ӳ���������SPI

__align(4) uint8_t g_DataTmpBuffer[0x1000] = {0};
#define SectorBuf	g_DataTmpBuffer

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: hal_spi_send_bytes
//	����˵��: SPI ��������,���������Ӳ��ͨ�ŷ�ʽ
//	��    ��: 	mode��ͨ�ŷ�ʽѡ��(0�����SPI;1��Ӳ��SPI)
//				pbdata���������ݵ��׵�ַ
//				send_length���������ݳ���
//	�� �� ֵ: ִ��״̬(true or false)
//	��    ��: 2020-03-12
//  ��    ע: �м���װ�ײ�ӿ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------------------------
void hal_spi_send_bytes(uint8_t mode, uint8_t *pbdata, uint16_t send_length)
{
    if (mode == 0)
    {
        for (uint16_t i = 0; i < send_length; i++)
        {
//            Spi_WriteByte(pbdata[i]);
        }
    }
    else if (mode == 1)
    {
        spi_master_send_some_bytes(1, pbdata, send_length);
		
//		for (uint16_t i = 0; i < send_length; i++)
//        {
//            spi_master_send_recv_byte(1, pbdata[i]);
//        }	
    }
    
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: hal_spi_recv_bytes
//	����˵��: SPI ��������,���������Ӳ��ͨ�ŷ�ʽ
//	��    ��: 	mode��ͨ�ŷ�ʽѡ��(0�����SPI;1��Ӳ��SPI)
//				pbdata���������ݵ��׵�ַ
//				send_length���������ݳ���
//	�� �� ֵ: ִ��״̬(true or false)
//	��    ��: 2020-03-12
//  ��    ע: �м���װ�ײ�ӿ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void hal_spi_recv_bytes(uint8_t mode, uint8_t *pbdata, uint16_t recv_length)
{
    if (mode == 0)
    {
        for (uint16_t i = 0; i < recv_length; i++)
        {
//             *pbdata++ = Spi_ReadByte();	//���ģ��SPI
        }   
		
    }
    else if (mode == 1)
    {
        spi_master_recv_some_bytes(1, pbdata, recv_length);	//Ӳ��SPI
		
//		for (uint16_t i = 0; i < recv_length; i++)
//        {
//            *pbdata++ = spi_master_send_recv_byte(1, 0xFF);
//        }
    }
    
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_flash_gpio_init
//	����˵��: SPI FLASH Ƭѡ�źų�ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_flash_gpio_init(void)
{
	GPIO_InitTypeDef gpio_config_init;

	//����CSӲ����ʽ
	RCC_AHB1PeriphClockCmd(RCC_PCLK_SPIM1_GPIO, ENABLE);		//����SPIM1 GPIOʱ��

	gpio_config_init.GPIO_Pin 	= FLASH_CS_IO;	//SPIM1_CLK_IO IO��ʼ��
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_config_init.GPIO_OType = GPIO_OType_PP;	//�������
	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;  
	gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPIM1_GPIO_PORT, &gpio_config_init);

	GPIO_SetBits(SPIM1_GPIO_PORT, FLASH_CS_IO);	//IO��ʼ״̬������Ϊ�ߵ�ƽ
}

#define HARD_SPI_PICOMM
#ifdef HARD_SPI_PICOMM	//Ӳ��SPI��־

#define SectorBuf  g_DataTmpBuffer
//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteEnable
//	����˵��: дʹ��,��λ WEL λ WEL λ(WEL-->1)
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteEnable(void)
{
	uint8_t command = FLASH_WRITE_ENABLE_CMD;

	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);//����дʹ��
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteDisable
//	����˵��: дʧ��,��λ WEL λ(WEL-->0)
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteDisable(void)
{
	uint8_t command = FLASH_WRITE_DISABLE_CMD;
	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	// Spi_WriteByte(FLASH_WRITE_DISABLE_CMD);	//����дʧ�� 04h
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSR
//	����˵��: ��״̬�Ĵ���
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: �����ڼ�� BUSY λ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t Flash_ReadSR(void)
{
	uint8_t ucTmpVal = 0;
	uint8_t command = FLASH_READ_SR_CMD;

	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);	//05h
	hal_spi_recv_bytes(SPI_COMM_MODE, &ucTmpVal, 1);

	// ucTmpVal = Spi_ReadByte();
	
	FLASH_CS_HIGH;
	
	return ucTmpVal;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSR
//	����˵��: д״̬�Ĵ���
//	��    ��: 	_ucByte:д��״̬�Ĵ�������ֵ
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteSR(uint8_t _ucByte)
{
	uint8_t command = FLASH_WRITE_SR_CMD;

	Flash_WriteEnable();	
	Flash_WaitNobusy();

	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);	//01h
	hal_spi_send_bytes(SPI_COMM_MODE, &_ucByte, 1);	//д��һ���ֽ�
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WaitNobusy
//	����˵��: ��� FLASH BUSY λ״̬
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ����Flash_ReadSR(),�ж�״̬�Ĵ�����R0λ,ִ�н�����������
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WaitNobusy(void)
{
	//FLASH_READ_SR_CMD ָ��ķ���,�е�FLASH���跢��һ��,FLASH�Զ��ظ�,�е�FLASH�޷��Զ��ظ�,��Ҫѭ��һֱ���͵ȴ�
	while(((Flash_ReadSR()) & 0x01)==0x01);	//�ȴ�BUSYλ���
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_FastReadByte
//	����˵��: flash ������(���ٶ�ȡ��Fast read operate at the highest poossible frequency)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulReadAddr: Ҫ����Flash���׵�ַ
//				_usNByte�� Ҫ�������ֽ���(���65535B)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ��_ulReadAddr��ַ,��������_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_ReadSomeBytes(uint8_t *ucpBuffer, uint32_t _ulReadAddr, uint16_t _usNByte)
{
	uint8_t command = FLASH_READ_DATA;
	uint8_t temp_buff[3] = {0};

	temp_buff[0] = (uint8_t)(_ulReadAddr >> 16);
	temp_buff[1] = (uint8_t)(_ulReadAddr >> 8);
	temp_buff[2] = (uint8_t)(_ulReadAddr >> 0);

	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[0], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[1], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[2], 1);

	hal_spi_recv_bytes(SPI_COMM_MODE, ucpBuffer, _usNByte);

	// Spi_WriteByte(FLASH_READ_DATA);	//������ȡ���� 03h
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>16));	//д��24λ��ַ
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>8));
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>0));

	// while(_usNByte--)
	// {
	// 	*ucpBuffer = Spi_ReadByte();
	// 	ucpBuffer++;
	// }
	
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_FastReadByte
//	����˵��: flash ������(���ٶ�ȡ��Fast read operate at the highest poossible frequency)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulReadAddr: Ҫ����Flash���׵�ַ
//				_usNByte�� Ҫ�������ֽ���(���65535B)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ��_ulReadAddr��ַ,��������_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_FastReadByte(uint8_t *ucpBuffer, uint32_t _ulReadAddr, uint16_t _usNByte)
{
	uint8_t command = FLASH_FASTREAD_DATA;
	uint8_t temp_buff[3] = {0};

	temp_buff[0] = (uint8_t)(_ulReadAddr >> 16);
	temp_buff[1] = (uint8_t)(_ulReadAddr >> 8);
	temp_buff[2] = (uint8_t)(_ulReadAddr >> 0);

	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[0], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[1], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[2], 1);

	hal_spi_recv_bytes(SPI_COMM_MODE, ucpBuffer, _usNByte);
	
	// Spi_WriteByte(FLASH_FASTREAD_DATA);//���ٶ�ȡ���� 0bh
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>16));//д��24λ��ַ
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>8));
	// Spi_WriteByte((uint8_t)(_ulReadAddr>>0));
	// Spi_WriteByte(0xFF);//�ȴ�8��ʱ��(dummy byte)
	// while(_usNByte--)
	// {
	// 	*ucpBuffer = Spi_ReadByte();
	// 	ucpBuffer++;
	// }
	
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WritePage
//	����˵��: flash д����(��ҳд��,һҳ256�ֽ�,д��֮ǰFLASH��ַ�ϱ���Ϊ0xFF)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WritePage(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	uint8_t command = FLASH_WRITE_PAGE;
	uint8_t temp_buff[3] = {0};

	temp_buff[0] = (uint8_t)(_ulWriteAddr >> 16);
	temp_buff[1] = (uint8_t)(_ulWriteAddr >> 8);
	temp_buff[2] = (uint8_t)(_ulWriteAddr >> 0);
	
	Flash_WriteEnable();	//дʹ��
	Flash_WaitNobusy();	//�ȴ�д�����
	
	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[0], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[1], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[2], 1);

	hal_spi_send_bytes(SPI_COMM_MODE, ucpBuffer, _usNByte);

	// Spi_WriteByte(FLASH_WRITE_PAGE);	//02h
	// Spi_WriteByte((uint8_t)(_ulWriteAddr>>16));	//д��24λ��ַ
	// Spi_WriteByte((uint8_t)(_ulWriteAddr>>8));
	// Spi_WriteByte((uint8_t)(_ulWriteAddr>>0));
	// while(_usNByte--)
	// {
	// 	Spi_WriteByte(*ucpBuffer);	//SPI д�뵥���ֽ�
	// 	ucpBuffer++;
	// }
	
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteNoCheck
//	����˵��: flash д����(��������,д��֮ǰ����ȷ��д�벿��FLASH������ȫΪ0xFf,����д��ʧ��)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽڣ������FLASH���ݼ��д��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteNoCheck(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	uint16_t PageByte = 256 - _ulWriteAddr % 256;//��ҳʣ���д�ֽ���

	if(_usNByte <= PageByte)	//������256�ֽ�
	{
		PageByte = _usNByte;
	}
	
	while(1)
	{
		Flash_WritePage(ucpBuffer, _ulWriteAddr, PageByte);
		if(_usNByte == PageByte)	//д�����
			break;
		else
		{
			ucpBuffer += PageByte;	//��һҳд�������
			_ulWriteAddr += PageByte;	//��һҳд��ĵ�ַ
			_usNByte -= PageByte;	//��д����ֽ����ݼ�
			if(_usNByte > 256)
			{
				PageByte = 256;
			}
			else
			{
				PageByte = _usNByte;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSomeBytes
//	����˵��: flash д����
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽڣ������FLASH���ݼ��д��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteSomeBytes(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	uint32_t ulSecPos = 0;				//�õ�����λ��
	uint16_t usSecOff = 0;				//����ƫ��
	uint16_t usSecRemain = 0;		//ʣ������
	uint32_t i = 0;

	ulSecPos = _ulWriteAddr / 4096;//��ַ��������(0--511)
	usSecOff = _ulWriteAddr % 4096;//�����ڵ�ַƫ��
	usSecRemain = 4096 - usSecOff;//������ȥƫ�ƣ���ʣ�����ֽ�

	if(_usNByte <= usSecRemain)	//д�����ݴ�С < ʣ�������ռ��С
	{
		usSecRemain = _usNByte;
	}

	while(1)
	{
		Flash_ReadSomeBytes(SectorBuf, ulSecPos*4096, 4096);//������������������
		for (i = 0; i < usSecRemain; i++)	//У������
		{
			if (SectorBuf[usSecOff + i] != 0xFF)//�������ݲ�Ϊ0xFF����Ҫ����
				break;
		}
		
		if(i < usSecRemain)	//��Ҫ����
		{
			Flash_EraseSector(ulSecPos);	//�����������
			for(i = 0; i < usSecRemain; i++)	//����д�������
			{
				SectorBuf[usSecOff + i] = ucpBuffer[i];
			}
			Flash_WriteNoCheck(SectorBuf, ulSecPos*4096, 4096);	//д����������(����=������+��д������)
		}
		else
		{
			Flash_WriteNoCheck(ucpBuffer, _ulWriteAddr, usSecRemain);//����Ҫ����,ֱ��д������
		}
		if(_usNByte == usSecRemain)	//д�����
		{
			Flash_WriteDisable();
			break;
		}
		else
		{
			ulSecPos++;		//������ַ����1
			usSecOff = 0;		//����ƫ�ƹ���
			ucpBuffer += usSecRemain;	//ָ��ƫ��
			_ulWriteAddr += usSecRemain;	//д��ַƫ��
			_usNByte -= usSecRemain;	//��д����ֽڵݼ�

			if(_usNByte > 4096)
			{
				usSecRemain = 4096;	//��д��һ����(4096�ֽڴ�С)
			}
			else
			{
				usSecRemain = _usNByte;		//��д������һ����������
			}
		}
		
	}
	
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ErasePage
//	����˵��: flash erase page
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: �е� FLASH ֧��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_ErasePage(uint32_t _ulPageAddr)
{
	_ulPageAddr *= 256;
	uint8_t temp_buff[3] = {0};
	uint8_t command = FLASH_ERASE_PAGE;
	
	temp_buff[0] = (uint8_t)(_ulPageAddr >> 16);
	temp_buff[1] = (uint8_t)(_ulPageAddr >> 8);
	temp_buff[2] = (uint8_t)(_ulPageAddr >> 0);
	
	Flash_WriteEnable();
	Flash_WaitNobusy();
	
	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[0], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[1], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[2], 1);
	
//	Spi_WriteByte(FLASH_ERASE_PAGE);	//ҳ����ָ��
//	Spi_WriteByte((uint8_t)(_ulPageAddr>>16));	//д��24λ��ַ
//	Spi_WriteByte((uint8_t)(_ulPageAddr>>8));
//	Spi_WriteByte((uint8_t)(_ulPageAddr>>0));
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseSector
//	����˵��: flash erase sector
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 1���� = 4K Bytes
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseSector(uint32_t _ulSectorAddr)
{
	uint8_t command = FLASH_ERASE_SECTOR;
	uint8_t temp_buff[3] = {0};
	
	temp_buff[0] = (uint8_t)(_ulSectorAddr >> 16);
	temp_buff[1] = (uint8_t)(_ulSectorAddr >> 8);
	temp_buff[2] = (uint8_t)(_ulSectorAddr >> 0);
	
	_ulSectorAddr *= 4096;	//1������ 4 KBytes
	
	Flash_WriteEnable();
	Flash_WaitNobusy();
	
	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[0], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[1], 1);
	hal_spi_send_bytes(SPI_COMM_MODE, &temp_buff[2], 1);


//	Spi_WriteByte(FLASH_ERASE_SECTOR);	//20h
//	Spi_WriteByte((uint8_t)(_ulSectorAddr>>16));	//д��24λ��ַ
//	Spi_WriteByte((uint8_t)(_ulSectorAddr>>8));
//	Spi_WriteByte((uint8_t)(_ulSectorAddr));
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseBlock
//	����˵��: flash erase block 
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 1�� = 64K Bytes
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseBlock(uint32_t _ulBlockAddr)
{
	uint8_t command = FLASH_ERASE_BLOCK;
	_ulBlockAddr *= 65536;	//���ַ,һ��64K
	
	Flash_WriteEnable();
	Flash_WaitNobusy();

	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	hal_spi_send_bytes(SPI_COMM_MODE, (uint8_t *)(_ulBlockAddr>>16), 1);
	hal_spi_send_bytes(SPI_COMM_MODE, (uint8_t *)(_ulBlockAddr>>8), 1);
	hal_spi_send_bytes(SPI_COMM_MODE, (uint8_t *)(_ulBlockAddr>>0), 1);

	// Spi_WriteByte(FLASH_ERASE_BLOCK);	//d8h
	// Spi_WriteByte((uint8_t)(_ulBlockAddr>>16));	//д��24λ��ַ
	// Spi_WriteByte((uint8_t)(_ulBlockAddr>>8));
	// Spi_WriteByte((uint8_t)(_ulBlockAddr));
	FLASH_CS_HIGH;

	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseChip
//	����˵��: flash erase chip , it makes flash  recovery FF
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseChip(void)
{
	uint8_t command = FLASH_ERASE_CHIP;

	Flash_WriteEnable();	//flashоƬдʹ��
	Flash_WaitNobusy();	//�ȴ�д�������
	
	FLASH_CS_LOW;
	hal_spi_recv_bytes(SPI_COMM_MODE, &command, 1);
	// Spi_WriteByte(FLASH_ERASE_CHIP);	//c7h
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_PowerDown
//	����˵��: flash into power down mode 
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_PowerDown(void)
{
	uint8_t command = FLASH_POWER_DOWN; 

	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	// Spi_WriteByte(FLASH_POWER_DOWN);	//b9h
	FLASH_CS_HIGH;
	delay_us(3);	// cs go high , need to delay 3us
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WakeUp
//	����˵��: wake up flash from power down mode or hign performance mode
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WakeUp(void)
{
	uint8_t command = FLASH_RELEASE_POWER_DOWN; 

	FLASH_CS_LOW;
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);
	// Spi_WriteByte(FLASH_RELEASE_POWER_DOWN);//abh
	FLASH_CS_HIGH;
	delay_us(3);	//CS go high , need delay 3us
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ReadDeviceID
//	����˵��: ��ȡFLASH ID(manufacturer ID-1Byte + Device ID-2Byte:type+density)
//	��    ��: ��
//	�� �� ֵ: ulJedId��FLASH ID 3�ֽ�
//	��    ��: 2020-03-06
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint16_t Flash_ReadDeviceID(void)
{
	uint8_t command = FLASH_READ_DEVICE_ID;
	uint16_t usFlashId = 0;
	uint8_t temp_buff[3] = {0};
	
	FLASH_CS_LOW;
	
	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);	//90h
	hal_spi_send_bytes(SPI_COMM_MODE, temp_buff, 3);	//д��24λ��ַ���ٵ�ַ
	hal_spi_recv_bytes(SPI_COMM_MODE, temp_buff, 2);

	// Spi_WriteByte(FLASH_READ_DEVICE_ID);	//90h
	// Spi_WriteByte(0x00);//д��24λ��ַ���ٵ�ַ
	// Spi_WriteByte(0x00);
	// Spi_WriteByte(0x00);	//���0x01,����� Device ID
	// usFlashId |= Spi_ReadByte()<<8;
	// usFlashId |= Spi_ReadByte();
	
	FLASH_CS_HIGH;
	
	usFlashId = (uint16_t)(temp_buff[0] << 8) | (temp_buff[1] << 0);

	return usFlashId;
}

//uint32_t spi_flash_read_id(void)
//{
//	uint32_t ulJedId = 0;
//	uint8_t recv_buff[5] = {0};
//	
//	 FLASH_CS_0();

//#if 0
//	 spi_master_send_recv_byte(1, FLASH_READ_JEDEC_ID);	//9fh
//	 ulJedId |= spi_master_send_recv_byte(1, 0xFF)<<16;
//	 ulJedId |= spi_master_send_recv_byte(1, 0xFF)<<8;
//	 ulJedId |= spi_master_send_recv_byte(1, 0xFF);
//#endif
//	
//#if 1
////	spi_master_send_recv_byte(1, FLASH_READ_JEDEC_ID);	//9fh
////	 
////	spi_master_recv_some_bytes(1, recv_buff, sizeof(recv_buff));
////	
////	ulJedId = (recv_buff[0] <<16) | (recv_buff[1] <<8) | (recv_buff[2]);
//	
////	spi_master_send_byte(1, 0xff);
////	recv_buff[0] = spi_master_recv_byte(1);
////	spi_master_send_byte(1, 0xff);
////	recv_buff[1] = spi_master_recv_byte(1);
////	spi_master_send_byte(1, 0xff);
////	recv_buff[2] = spi_master_recv_byte(1);
////	spi_master_send_byte(1, 0xff);
////	recv_buff[3] = spi_master_recv_byte(1);
////	spi_master_send_byte(1, 0xff);
////	recv_buff[4] = spi_master_recv_byte(1);
//	
//	for (uint16_t i = 0; i < sizeof(recv_buff); i++)
//	{
//		printf("recv ---> %d 0x%02X \r\n", i, recv_buff[i]);
//	}
//	printf("\r\n");
//	
//#endif

//	 FLASH_CS_1();
//	
//	return ulJedId;
//}
 
//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ReadJEDECID
//	����˵��: ��ȡFLASH ID(manufacturer ID-1Byte + Device ID-2Byte:type+density)
//	��    ��: ��
//	�� �� ֵ: ulJedId��FLASH ID 3�ֽ�
//	��    ��: 2020-03-06
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint32_t Flash_ReadJEDECID(void)
{
	uint8_t command = FLASH_READ_JEDEC_ID;
	uint32_t flash_jed_id = 0;
	uint8_t recv_buff[3] = {0};
	
	FLASH_CS_LOW;

	hal_spi_send_bytes(SPI_COMM_MODE, &command, 1);	//9fh
	hal_spi_recv_bytes(SPI_COMM_MODE, recv_buff, 3);
	
	FLASH_CS_HIGH;

	flash_jed_id = (recv_buff[0] << 16) | (recv_buff[1] << 8) | (recv_buff[2] << 0);
	
	return flash_jed_id;
}







#endif	//SOFT_SPI_COMM
//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------

#if 0

//#ifdef SOFT_SPI_COMM	//���ģ��SPI��־

#define SectorBuf  g_DataTmpBuffer

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteEnable
//	����˵��: дʹ��,��λ WEL λ WEL λ(WEL-->1)
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteEnable(void)
{
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_WRITE_ENABLE_CMD);//����дʹ��
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteDisable
//	����˵��: дʧ��,��λ WEL λ(WEL-->0)
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteDisable(void)
{
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_WRITE_DISABLE_CMD);	//����дʧ�� 04h
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSR
//	����˵��: ��״̬�Ĵ���
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-07
//  ��    ע: �����ڼ�� BUSY λ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t Flash_ReadSR(void)
{
	uint8_t ucTmpVal = 0;
	
	FLASH_CS_LOW;
	
	Spi_WriteByte(FLASH_READ_SR_CMD);	//05h
	ucTmpVal = Spi_ReadByte();
	
	FLASH_CS_HIGH;
	
	return ucTmpVal;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSR
//	����˵��: д״̬�Ĵ���
//	��    ��: 	_ucByte:д��״̬�Ĵ�������ֵ
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteSR(uint8_t _ucByte)
{
	Flash_WriteEnable();	
	Flash_WaitNobusy();

	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_WRITE_SR_CMD);	//01h
	Spi_WriteByte(_ucByte);	//д��һ���ֽ�
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WaitNobusy
//	����˵��: ��� FLASH BUSY λ״̬
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ����Flash_ReadSR(),�ж�״̬�Ĵ�����R0λ,ִ�н�����������
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WaitNobusy(void)
{
	//FLASH_READ_SR_CMD ָ��ķ���,�е�FLASH���跢��һ��,FLASH�Զ��ظ�,�е�FLASH�޷��Զ��ظ�,��Ҫѭ��һֱ���͵ȴ�
	while(((Flash_ReadSR()) & 0x01)==0x01);	//�ȴ�BUSYλ���
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_FastReadByte
//	����˵��: flash ������(���ٶ�ȡ��Fast read operate at the highest poossible frequency)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulReadAddr: Ҫ����Flash���׵�ַ
//				_usNByte�� Ҫ�������ֽ���(���65535B)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ��_ulReadAddr��ַ,��������_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_ReadSomeBytes(uint8_t *ucpBuffer, uint32_t _ulReadAddr, uint16_t _usNByte)
{
	FLASH_CS_LOW;
	
	Spi_WriteByte(FLASH_READ_DATA);	//������ȡ���� 03h
	Spi_WriteByte((uint8_t)(_ulReadAddr>>16));	//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulReadAddr>>8));
	Spi_WriteByte((uint8_t)(_ulReadAddr>>0));
	while(_usNByte--)
	{
		*ucpBuffer = Spi_ReadByte();
		ucpBuffer++;
	}
	
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_FastReadByte
//	����˵��: flash ������(���ٶ�ȡ��Fast read operate at the highest poossible frequency)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulReadAddr: Ҫ����Flash���׵�ַ
//				_usNByte�� Ҫ�������ֽ���(���65535B)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ��_ulReadAddr��ַ,��������_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_FastReadByte(uint8_t *ucpBuffer, uint32_t _ulReadAddr, uint16_t _usNByte)
{
	FLASH_CS_LOW;
	
	Spi_WriteByte(FLASH_FASTREAD_DATA);//���ٶ�ȡ���� 0bh
	Spi_WriteByte((uint8_t)(_ulReadAddr>>16));//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulReadAddr>>8));
	Spi_WriteByte((uint8_t)(_ulReadAddr>>0));
	Spi_WriteByte(0xFF);//�ȴ�8��ʱ��(dummy byte)
	while(_usNByte--)
	{
		*ucpBuffer = Spi_ReadByte();
		ucpBuffer++;
	}
	
	FLASH_CS_HIGH;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WritePage
//	����˵��: flash д����(��ҳд��,һҳ256�ֽ�,д��֮ǰFLASH��ַ�ϱ���Ϊ0xFF)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽ�
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WritePage(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	Flash_WriteEnable();	//дʹ��
	Flash_WaitNobusy();	//�ȴ�д�����
	
	FLASH_CS_LOW;
	
	Spi_WriteByte(FLASH_WRITE_PAGE);	//02h
	Spi_WriteByte((uint8_t)(_ulWriteAddr>>16));	//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulWriteAddr>>8));
	Spi_WriteByte((uint8_t)(_ulWriteAddr>>0));
	while(_usNByte--)
	{
		Spi_WriteByte(*ucpBuffer);	//SPI д�뵥���ֽ�
		ucpBuffer++;
	}
	
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteNoCheck
//	����˵��: flash д����(��������,д��֮ǰ����ȷ��д�벿��FLASH������ȫΪ0xFf,����д��ʧ��)
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽڣ������FLASH���ݼ��д��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteNoCheck(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	uint16_t PageByte = 256 - _ulWriteAddr % 256;//��ҳʣ���д�ֽ���

	if(_usNByte <= PageByte)	//������256�ֽ�
	{
		PageByte = _usNByte;
	}
	
	while(1)
	{
		Flash_WritePage(ucpBuffer, _ulWriteAddr, PageByte);
		if(_usNByte == PageByte)	//д�����
			break;
		else
		{
			ucpBuffer += PageByte;	//��һҳд�������
			_ulWriteAddr += PageByte;	//��һҳд��ĵ�ַ
			_usNByte -= PageByte;	//��д����ֽ����ݼ�
			if(_usNByte > 256)
			{
				PageByte = 256;
			}
			else
			{
				PageByte = _usNByte;
			}
		}
	}
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WriteSomeBytes
//	����˵��: flash д����
//	��    ��: 	ucpBuffer�����ݴ洢���׵�ַ
//				_ulWriteAddr: Ҫ��д��Flash���׵�ַ
//				_usNByte�� Ҫд����ֽ���(���65535B = 64K ��)
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: _ulWriteAddr,����д��_usNByte���ȵ��ֽڣ������FLASH���ݼ��д��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WriteSomeBytes(uint8_t *ucpBuffer, uint32_t _ulWriteAddr, uint16_t _usNByte)
{
	uint32_t ulSecPos = 0;				//�õ�����λ��
	uint16_t usSecOff = 0;				//����ƫ��
	uint16_t usSecRemain = 0;		//ʣ������
	uint32_t i = 0;

	ulSecPos = _ulWriteAddr / 4096;//��ַ��������(0--511)
	usSecOff = _ulWriteAddr % 4096;//�����ڵ�ַƫ��
	usSecRemain = 4096 - usSecOff;//������ȥƫ�ƣ���ʣ�����ֽ�

	if(_usNByte <= usSecRemain)	//д�����ݴ�С < ʣ�������ռ��С
	{
		usSecRemain = _usNByte;
	}

	while(1)
	{
		Flash_ReadSomeBytes(SectorBuf, ulSecPos*4096, 4096);//������������������
		for (i = 0; i < usSecRemain; i++)	//У������
		{
			if (SectorBuf[usSecOff + i] != 0xFF)//�������ݲ�Ϊ0xFF����Ҫ����
				break;
		}
		
		if(i < usSecRemain)	//��Ҫ����
		{
			Flash_EraseSector(ulSecPos);	//�����������
			for(i = 0; i < usSecRemain; i++)	//����д�������
			{
				SectorBuf[usSecOff + i] = ucpBuffer[i];
			}
			Flash_WriteNoCheck(SectorBuf, ulSecPos*4096, 4096);	//д����������(����=������+��д������)
		}
		else
		{
			Flash_WriteNoCheck(ucpBuffer, _ulWriteAddr, usSecRemain);//����Ҫ����,ֱ��д������
		}
		if(_usNByte == usSecRemain)	//д�����
		{
			Flash_WriteDisable();
			break;
		}
		else
		{
			ulSecPos++;		//������ַ����1
			usSecOff = 0;		//����ƫ�ƹ���
			ucpBuffer += usSecRemain;	//ָ��ƫ��
			_ulWriteAddr += usSecRemain;	//д��ַƫ��
			_usNByte -= usSecRemain;	//��д����ֽڵݼ�

			if(_usNByte > 4096)
			{
				usSecRemain = 4096;	//��д��һ����(4096�ֽڴ�С)
			}
			else
			{
				usSecRemain = _usNByte;		//��д������һ����������
			}
		}
		
	}
	
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ErasePage
//	����˵��: flash erase page
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: �е� FLASH ֧��
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_ErasePage(uint32_t _ulPageAddr)
{
	_ulPageAddr *= 256;
	
	Flash_WriteEnable();
	Flash_WaitNobusy();
	
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_ERASE_PAGE);	//ҳ����ָ��
	Spi_WriteByte((uint8_t)(_ulPageAddr>>16));	//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulPageAddr>>8));
	Spi_WriteByte((uint8_t)(_ulPageAddr>>0));
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseSector
//	����˵��: flash erase sector
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 1���� = 4K Bytes
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseSector(uint32_t _ulSectorAddr)
{
	_ulSectorAddr *= 4096;	//1������ 4 KBytes
	
	Flash_WriteEnable();
	Flash_WaitNobusy();
	
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_ERASE_SECTOR);	//20h
	Spi_WriteByte((uint8_t)(_ulSectorAddr>>16));	//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulSectorAddr>>8));
	Spi_WriteByte((uint8_t)(_ulSectorAddr));
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseBlock
//	����˵��: flash erase block 
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: 1�� = 64K Bytes
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseBlock(uint32_t _ulBlockAddr)
{
	_ulBlockAddr *= 65536;	//���ַ,һ��64K
	
	Flash_WriteEnable();
	Flash_WaitNobusy();

	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_ERASE_BLOCK);	//d8h
	Spi_WriteByte((uint8_t)(_ulBlockAddr>>16));	//д��24λ��ַ
	Spi_WriteByte((uint8_t)(_ulBlockAddr>>8));
	Spi_WriteByte((uint8_t)(_ulBlockAddr));
	FLASH_CS_HIGH;

	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_EraseChip
//	����˵��: flash erase chip , it makes flash  recovery FF
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_EraseChip(void)
{
	Flash_WriteEnable();	//flashоƬдʹ��
	Flash_WaitNobusy();	//�ȴ�д�������
	
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_ERASE_CHIP);	//c7h
	FLASH_CS_HIGH;
	
	Flash_WaitNobusy();	//�ȴ�д�����
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_PowerDown
//	����˵��: flash into power down mode 
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_PowerDown(void)
{
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_POWER_DOWN);	//b9h
	FLASH_CS_HIGH;
	Sys_delay_us(3);	// cs go high , need to delay 3us
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_WakeUp
//	����˵��: wake up flash from power down mode or hign performance mode
//	��    ��: no
//	�� �� ֵ: no
//	��    ��: 2020-03-07
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void Flash_WakeUp(void)
{
	FLASH_CS_LOW;
	Spi_WriteByte(FLASH_RELEASE_POWER_DOWN);//abh
	FLASH_CS_HIGH;
	Sys_delay_us(3);	//CS go high , need delay 3us
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ReadDeviceID
//	����˵��: ��ȡFLASH ID(manufacturer ID-1Byte + Device ID-2Byte:type+density)
//	��    ��: ��
//	�� �� ֵ: ulJedId��FLASH ID 3�ֽ�
//	��    ��: 2020-03-06
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint16_t Flash_ReadDeviceID(void)
{
	uint16_t usFlashId = 0;
	
	FLASH_CS_LOW;
	
	Spi_WriteByte(FLASH_READ_DEVICE_ID);	//90h
	Spi_WriteByte(0x00);//д��24λ��ַ���ٵ�ַ
	Spi_WriteByte(0x00);
	Spi_WriteByte(0x00);	//���0x01,����� Device ID
	usFlashId |= Spi_ReadByte()<<8;
	usFlashId |= Spi_ReadByte();
	
	FLASH_CS_HIGH;
	
	return usFlashId;
}
 
//--------------------------------------------------------------------------------------------------------
//	�� �� ��: Flash_ReadJEDECID
//	����˵��: ��ȡFLASH ID(manufacturer ID-1Byte + Device ID-2Byte:type+density)
//	��    ��: ��
//	�� �� ֵ: ulJedId��FLASH ID 3�ֽ�
//	��    ��: 2020-03-06
//  ��    ע: ���ģ��SPI
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint32_t Flash_ReadJEDECID(void)
{
	uint32_t ulJedId = 0;
	
	FLASH_CS_LOW;

	Spi_WriteByte(FLASH_READ_JEDEC_ID);	//9fh
	ulJedId |= Spi_ReadByte()<<16;
	ulJedId |= Spi_ReadByte()<<8;
	ulJedId |= Spi_ReadByte();
	
	FLASH_CS_HIGH;
	
	return ulJedId;
}

#endif	//0

#endif	//__PRJ_STM32F40X_DEVEXFLASH_C__
