#ifndef __PRJ_STM32F10X_HARDWARE_SPI_C__
#define __PRJ_STM32F10X_HARDWARE_SPI_C__

#include "hardware_spi.h"
#include "drvexflash.h"
#include <stdio.h>

#define HARD_SPI_COMM

#ifdef HARD_SPI_COMM

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ������SD Card/W25X16/24L01/JF24C							  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_gpio_init
//	����˵��: SPI Ӳ��IO��ʼ��
//	��    ��: 	spi_chl��SPIM ͨ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_gpio_init(uint8_t spi_chl)
{
	GPIO_InitTypeDef gpio_config_init;

	if (spi_chl == 1)
	{
		RCC_AHB1PeriphClockCmd(RCC_PCLK_SPIM1_GPIO, ENABLE);		//����SPIM1 GPIOʱ��
		
		//SP1��Ӧ���Ÿ���ӳ��
		GPIO_PinAFConfig(SPIM1_GPIO_PORT, GPIO_PinSource5, GPIO_AF_SPI1); //GPIOA5����ΪSPI1
		GPIO_PinAFConfig(SPIM1_GPIO_PORT, GPIO_PinSource6, GPIO_AF_SPI1); //GPIOA6����ΪSPI1
		GPIO_PinAFConfig(SPIM1_GPIO_PORT, GPIO_PinSource7, GPIO_AF_SPI1); //GPIOA7����ΪSPI1
		
//		gpio_config_init.GPIO_Pin 		= SPIM1_CLK_IO | SPIM1_MISO_IO | SPIM1_MOSI_IO;	//SPIM1_CLK_IO IO��ʼ��
		gpio_config_init.GPIO_Pin 		= SPIM1_CLK_IO | SPIM1_MOSI_IO;
		gpio_config_init.GPIO_Mode 		= GPIO_Mode_AF;  //�����������
		gpio_config_init.GPIO_Speed 	= GPIO_Speed_50MHz;
		gpio_config_init.GPIO_OType 	= GPIO_OType_PP;
		gpio_config_init.GPIO_PuPd		= GPIO_PuPd_UP;
		GPIO_Init(SPIM1_GPIO_PORT, &gpio_config_init);
		
		gpio_config_init.GPIO_Pin 		= SPIM1_MISO_IO;	//SPIM1_MISO_IO IO��ʼ��
		gpio_config_init.GPIO_Mode 		= GPIO_Mode_IN;	//��������	
		gpio_config_init.GPIO_PuPd		= GPIO_PuPd_NOPULL;
		gpio_config_init.GPIO_Mode 		= GPIO_Mode_AF;	//���ù��ܱ�������
		gpio_config_init.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_Init(SPIM1_GPIO_PORT, &gpio_config_init);

		GPIO_SetBits(SPIM1_GPIO_PORT, SPIM1_CLK_IO | SPIM1_MISO_IO | SPIM1_MOSI_IO);	//IO��ʼ״̬������Ϊ�ߵ�ƽ
	}		
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_init
//	����˵��: SPI Ӳ�����ò�����ʼ��
//	��    ��: 	spi_chl��SPIM ͨ��
//	�� �� ֵ: ��
//	��    ��: 2021-05-27
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_master_init(uint8_t spi_chl)
{
	SPI_InitTypeDef  spi_config_init;
#if 1  
	if(spi_chl == 1)
	{	
//		spi_flash_gpio_init();	//spi flash cs ��ʼ��
//		sd_gpio_init();	//spi sd cs ��ʼ��
//		nrf24l01_gpio_init();//spi nrf24l01 cs ��ʼ��
		
		spi_gpio_init(1);	//spi gpio ��ʼ��

		RCC_APB2PeriphClockCmd(RCC_PCLK_SPIM1_HD, ENABLE);	//SPI1ʱ��ʹ��

		spi_config_init.SPI_Direction 			= SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
		spi_config_init.SPI_Mode 				= SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
		spi_config_init.SPI_DataSize 			= SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
		spi_config_init.SPI_CPOL 				= SPI_CPOL_Low;		//ѡ���˴���ʱ�ӵ���̬:����ʱ�ӵ�
		spi_config_init.SPI_CPHA 				= SPI_CPHA_1Edge;	//���ݲ���(����)�ڵ�1��ʱ����
		spi_config_init.SPI_NSS					= SPI_NSS_Soft;//SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
		spi_config_init.SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
		spi_config_init.SPI_FirstBit 			= SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
		spi_config_init.SPI_CRCPolynomial 		= 7;	//CRCֵ����Ķ���ʽ
		
		SPI_Init(SPI1, &spi_config_init);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	 
		SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
		
//		spi_master_send_recv_byte(1, 0xFF);	//��������	
	
	}
#endif
} 

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_send_recv_byte
//	����˵��: SPI �շ�����
//	��    ��: 	spi_chl��SPIM ͨ��
//				send_byte�����͵�����
//	�� �� ֵ: ��
//	��    ��: 2020-03-14
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t spi_master_send_recv_byte(uint8_t spi_chl, uint8_t spi_byte)
{		
	uint8_t time = 0;
	
	if (spi_chl == 1)			    
	{
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
			time++;
			if(time > 200)
			{
				return 1;
			}
		}			  
		SPI_I2S_SendData(SPI1, spi_byte); //ͨ������SPIx����һ������
	
		time = 0;

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
			time++;
			if(time > 200)
			{
				return 1;
			}
		}	  						    
			return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����	
	}
	else 
	{
		return 1;
	}
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_speed_set
//	����˵��: SPI ͨ���ٶ�����
//	��    ��: 	spi_chl��SPIͨ��
//				spi_speed�� ͨ���ٶ�
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע: 
//			SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//			SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//			SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//			SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_master_speed_set(uint8_t spi_chl, uint8_t spi_speed)
{
	SPI_InitTypeDef  spi_config_init;

	if (spi_chl == 1)
	{
		spi_config_init.SPI_BaudRatePrescaler = spi_speed ;

		SPI_Init(SPI1, &spi_config_init);
		SPI_Cmd(SPI1, ENABLE);
	}
} 
#if 0
//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_send_byte
//	����˵��: SPI ����һ���ֽ�����
//	��    ��: 	spi_chl��SPIM ͨ��
//				send_byte�����͵�����
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t spi_master_send_byte(uint8_t spi_chl, uint8_t send_byte)
{		
	
	if (spi_chl == 1)
	{
		spi_master_send_recv_byte(spi_chl, send_byte);
		
	}
	
	return true;
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_recv_byte
//	����˵��: SPI ����һ���ֽ�����
//	��    ��: 	spi_chl��SPIM ͨ��
//	�� �� ֵ: ���յ�����
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
uint8_t spi_master_recv_byte(uint8_t spi_chl)
{
	
	if (spi_chl == 1)
	{
		spi_master_send_recv_byte(spi_chl, 0xFF);
		
	}
	
	return true;
}
#endif	//spi���շ���ͬʱ���е�,���ҷ��ͷ�ҪΪ���շ��ṩʱ��,�շ��ֿ��ᵼ�´��������ʱ�������

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_send_some_bytes
//	����˵��: SPI ���Ͷ���ֽ�����
//	��    ��: 	spi_chl��SPIM ͨ��
//				pbdata�����͵������׵�ַ
//				send_length���������ݳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_master_send_some_bytes(uint8_t spi_chl, uint8_t *pbdata, uint16_t send_length)
{
	uint16_t i = 0;

	for (i = 0; i < send_length; i++)
	{
		spi_master_send_recv_byte(spi_chl, pbdata[i]);
	}
	
//	while (send_length--)
//	{
//		spi_master_send_byte(spi_chl, *pbdata++);
//	}
	
}

//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_recv_some_bytes
//	����˵��: SPI ���ն���ֽ�����
//	��    ��: 	spi_chl��SPIM ͨ��
//				pbdata�����յ������׵�ַ
//				send_length���������ݳ���
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_master_recv_some_bytes(uint8_t spi_chl, uint8_t *pbdata, uint16_t recv_length)
{
	uint8_t *temp_data = pbdata;

	while (recv_length--)
	{
		*temp_data++ = spi_master_send_recv_byte(spi_chl, 0xFF);	//���� 0xff Ϊ���豸�ṩʱ��
	}
	
}


//--------------------------------------------------------------------------------------------------------
//	�� �� ��: spi_master_nvic_set
//	����˵��: SPI �ж�����
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-03-12
//  ��    ע:���� Unix like ��ʽ
//	��    ��: by ����AI
//--------------------------------------------------------------------------------------------------------
void spi_master_nvic_set(void)
{
	NVIC_InitTypeDef nvic_config_init;
	
	nvic_config_init.NVIC_IRQChannel 						= SPI1_IRQn;
	nvic_config_init.NVIC_IRQChannelPreemptionPriority 	= 2;
	nvic_config_init.NVIC_IRQChannelSubPriority 			= 2;
	nvic_config_init.NVIC_IRQChannelCmd 					= ENABLE;

	NVIC_Init(&nvic_config_init);
	
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);	//����SPI1���豸����ʹ��
}


//-------------------------------------------------------------------------------------------------------
//	Author:By ����AI
//	Date:2019.12.22
//	Name: SPI1_IRQHandler
//	Function:SPI1 �жϷ�����
//	Parameter:  ��
//	Rerurn: ��
//	Note:
//		(1)g_SpiTxRxLen:ȫ�ֱ���,spi�������ݳ���
//		(2)g_byTmpBuffer:ȫ�ֱ���,spi���յ������ݴ�
//-------------------------------------------------------------------------------------------------------
//void SPI1_IRQHandler(void)
//{
//	uint16_t i = 0;
//	
//	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)	//���������ж�
//	{
//		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);	//����жϱ�־
//		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, DISABLE);	//��ֹ�ж�ʹ��
//		for (i = 0; i < g_SpiTxRxLen; i++)
//		{
////			while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
//			g_DataTmpBuffer[i] = (uint8_t)SPI_I2S_ReceiveData(SPI1);	//��������
//		}
//		SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);	//����ʹ���ж�
//	}
//}




uint8_t SPI_WriteReadByte(uint8_t TxData)
{
  while((SPI1->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
  SPI1->DR = TxData;

  while((SPI1->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
  return SPI1->DR;
}

uint16_t SFLASH_ReadID(void)
{
  uint16_t ID = 0;
   FLASH_CS_0();                              //ʹ������

//  SPI_WriteReadByte(90);           //���豸ID��ָ��
//  SPI_WriteReadByte(0x00);
//  SPI_WriteReadByte(0x00);
//  SPI_WriteReadByte(0x00);
	
//	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
//	SPI1_ReadWriteByte(0x00); 	    
//	SPI1_ReadWriteByte(0x00); 	    
//	SPI1_ReadWriteByte(0x00); 

//  ID |= SPI_WriteReadByte(0xFF)<<8;              //��ȡID
//  ID |= SPI_WriteReadByte(0xFF);
	

//	spi_master_send_byte(1, 0x90);//���Ͷ�ȡID����	    
//	spi_master_send_byte(1, 0x00); 	    
//	spi_master_send_byte(1, 0x00); 	    
//	spi_master_send_byte(1, 0x00); 

//	ID |= spi_master_recv_byte(0xFF)<<8;              //��ȡID
//	ID |= spi_master_recv_byte(0xFF);

	spi_master_send_recv_byte(1, 0x90);//���Ͷ�ȡID����	    
	spi_master_send_recv_byte(1, 0x00); 	    
	spi_master_send_recv_byte(1, 0x00); 	    
	spi_master_send_recv_byte(1, 0x00); 

	ID |= spi_master_send_recv_byte(1, 0xFF)<<8;              //��ȡID
	ID |= spi_master_send_recv_byte(1, 0xFF);
  
   FLASH_CS_1();                             //ʧ������
	
  return ID;
}

uint8_t SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����					    
}

uint16_t SPI_Flash_ReadID(void)
{
	u16 Temp = 0;	  
	 FLASH_CS_0();			    
	SPI1_ReadWriteByte(0x90);//���Ͷ�ȡID����	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	    
	SPI1_ReadWriteByte(0x00); 	 			   
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;  
	Temp|=SPI1_ReadWriteByte(0xFF);	 
	 FLASH_CS_1();				    
	return Temp;
}  


#endif	//HARD_SPI_COMM

#endif	//__PRJ_STM32F10X_HARDWARE_SPI_C__
