//--------------------------------------------------------------------------------------------------------
//
//	ģ������ : OLED����
//	�ļ����� : drvoled.c
//	��    �� : V1.0
//	˵    �� : 
//				(1) drvoled.c����оƬSpiͨ�Žӿ�
//				(2) 
//				(3) 
//				(4) 
//
//	�޸ļ�¼ :
//		�汾��     ����        ����     ˵��
//		V1.0    2020-04-11  ����AI  ��ʽ����
//		V1.1    
//		V1.2	
//		V1.3	
//
//	Copyright (C), 2020-2030, ΢�Ź��ںš���TECHTIMES
//
//--------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F10X_DRVOLED_C__
#define __PRJ_STM32F10X_DRVOLED_C__

#include "drvoled.h"
#include "oled_font.h"
#include "bmp.h"
#include "hardware_spi.h"
#include "delay.h"


//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_gpio_init
//	����˵��: oled gpio ��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-05-27
//  ��    ע: 
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
static void oled_gpio_init(void)
{
    GPIO_InitTypeDef gpio_config_init;

	RCC_AHB1PeriphClockCmd(OLED_GPIO_CLK, ENABLE);		//����OLED GPIOʱ��

	gpio_config_init.GPIO_Pin = OLED_CS_PIN | OLED_DC_PIN | OLED_RST_PIN;	//OLED IO��ʼ��
	gpio_config_init.GPIO_Mode 	= GPIO_Mode_OUT;	//���
	gpio_config_init.GPIO_OType = GPIO_OType_PP;	//�������
	gpio_config_init.GPIO_PuPd	= GPIO_PuPd_UP;  
	gpio_config_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_PORT, &gpio_config_init);

	GPIO_SetBits(OLED_PORT, OLED_CS_PIN | OLED_DC_PIN | OLED_RST_PIN);	//IO��ʼ״̬������Ϊ�ߵ�ƽ
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_write_byte
//	����˵��: oled дһ�ֽ�����
//	��    ��: write_byte��д�������
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ����SPI�������ݺ���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
static void oled_write_byte(uint8_t write_byte)
{
//	hal_spi_send_bytes(1, &write_byte, 1);	//Ӳ����ʽSPI(�м�㺯����װ)
	spi_master_send_recv_byte(1, write_byte);	//�ײ�Ӳ��SPI
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_write_operate
//	����˵��: oled д����
//	��    ��: mode��д��������ݻ�������(0������;1������)
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ����д���ݺ�д����,ͨ��OLED DC ���Ÿߵ͵�ƽ����
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
static void oled_write_operate(uint8_t mode, uint8_t dat)
{
	OLED_CS_LOW;
	
	if (mode)	//д������
	{
		OLED_DC_HIGH;
	}
	else 	//д������
	{
		OLED_DC_LOW;
	}
	
	oled_write_byte(dat);
	OLED_CS_HIGH;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_set_pos
//	����˵��: oled ��ʾ��������
//	��    ��: x:������;y��������
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_set_pos(uint8_t x, uint8_t y)
{
	oled_write_operate(OLED_COMM, 0xb0 + y);	//����ҳ(page)��ַ
	oled_write_operate(OLED_COMM, ((x & 0xf0) >> 4) | 0x10);	//�и���λ��ַ
	oled_write_operate(OLED_COMM, (x & 0x0f) | 0x01);	//�е���λ��ַ
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_on
//	����˵��: oled ����ʾ
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: д��AFָ���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_on(void)
{
	oled_write_operate(OLED_COMM, 0x8d);	//��DCDC
	oled_write_operate(OLED_COMM, 0x14);	//��DCDC
	oled_write_operate(OLED_COMM, 0xaf);	//��OLED
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_off
//	����˵��: oled �ر���ʾ
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: д��AEָ��ر�
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_off(void)
{
	oled_write_operate(OLED_COMM, 0x8d);//����DCDC
	oled_write_operate(OLED_COMM, 0x10);//�ر�DCDC
	oled_write_operate(OLED_COMM, 0xae);//�ر�OLED
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_clear
//	����˵��: oled �����ʾ
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: д��00���
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_clear(void)
{
	uint8_t page = 0;
	uint8_t i = 0;
	
	for (page = 0; page < 8; page++)
	{
		oled_write_operate(OLED_COMM, 0xb0 + page);	//����ҳ��ַ(0--7)
		oled_write_operate(OLED_COMM, 0x00);	//������ʾλ�á��е͵�ַ
		oled_write_operate(OLED_COMM, 0x10); 	//������ʾλ�á��иߵ�ַ
	
		for(i = 0; i < 128; i++)
		{
			oled_write_operate(OLED_DATA, 0x00);	//0x00����
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_set
//	����˵��: oled ��ʾ����������
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: д��ff����
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_set(void)
{
	uint8_t page = 0;
	uint8_t i = 0;
	
	for (page = 0; page < 8; page++)
	{
		oled_write_operate(OLED_COMM, 0xb0 + page);	//����ҳ��ַ(0--7)
		oled_write_operate(OLED_COMM, 0x00);	//������ʾλ�á��е͵�ַ
		oled_write_operate(OLED_COMM, 0x10);	//������ʾλ�á��иߵ�ַ
	
		for (i = 0; i < 128; i++)
		{
			oled_write_operate(OLED_DATA, 0xff);
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_one_char
//	����˵��: oled ��ʾ�����ַ�
//	��    ��: x��������(��);y:������(��);str����ʾ�ַ�
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_one_char(uint8_t x, uint8_t y, uint8_t str)
{
	uint8_t i = 0;
	uint8_t ret = 0;
	//ret = str -32;
	ret = str - ' ';//�õ�ƫ�ƺ��ֵ,��ASCLL�����һ������.���ڶ�ά������������λ��  
	
	if (x > (MAX_COLUMN - 1))	//�г������,��������һ��
	{
		x = 0;
		if (SIZE == 8 )
		{
			y = y + 1;//���8�ŵ��ַ�
		}
		if (SIZE == 16 )
		{
			y = y + 2;//���16�ŵ��ַ�
		}
	}
	if (SIZE == 16 )
	{
		oled_set_pos(x, y);
		//16������ֳ�������д��
		for (i = 0; i < 8; i++)
		{
			oled_write_operate(OLED_DATA, F8X16[ret*16+i]);
		}
		
		oled_set_pos(x, y + 1);
		for	(i = 0; i < 8; i++)
		{
			oled_write_operate(OLED_DATA, F8X16[ret*16+i+8]);
		}
	}
	else 
	{
		oled_set_pos(x, y + 1);
		for(i = 0; i < 6; i++)
		{
			oled_write_operate(OLED_DATA, F6x8[ret][i]);
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_str
//	����˵��: oled ��ʾ����ַ�
//	��    ��: x��������(��);y:������(��);str����ʾ�ַ���
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_str(uint8_t x, uint8_t y, uint8_t *str)
{
	uint8_t i = 0;
	
	while(str[i] != '\0')
	{
		oled_dis_one_char(x, y, str[i]);
		x += 8;
		
		if(x > 120)
		{
			x = 0;
			y += 2;
		}
		i++;
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_chinese
//	����˵��: oled ��ʾ����
//	��    ��: x��������(��);y:������(��);no����ʾ����λ�ڱ���λ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_chinese(uint8_t x, uint8_t y, uint8_t no)
{
	uint8_t i = 0;
	uint8_t addr = 0;
	
	oled_set_pos(x, y);
	for (i = 0; i < 16; i++)//��������Ѱַ
	{
		oled_write_operate(OLED_DATA, TEST[2*no][i]);
		addr += 1;
	}
	
	oled_set_pos(x, y + 1);
	for (i = 0; i < 16; i++)
	{
		oled_write_operate(OLED_DATA, TEST[2*no+1][i]);
		addr += 1;
	}
	
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_picture
//	����˵��: oled ��ʾͼƬ
//	��    ��: x��������(��);y:������(��);no����ʾ����λ�ڱ���λ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_picture(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *bmp)
{
	uint8_t x = 0;
	uint8_t y = 0;
	uint32_t i = 0;
	
	if ((y1 % 8) == 0)
	{
		y = y1 / 8;
	}
	else
	{
		y = y1 /8 + 1;
	}
	
	for (y = y0; y < y1; y++)	//ÿ��(0 - 7 page)
	{
		oled_set_pos(x0, y);
		
		for (x = x0; x < x1; x++)	//0~128
		{
			oled_write_operate(OLED_DATA, bmp[i++]);
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_logo
//	����˵��: oled ��ʾͼƬlogo
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ����Ļ��ʾ
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_logo(void)
{
	const uint8_t *pdata = dis_tmp_buff;
	uint8_t page = 0;
	uint8_t seg = 0;
	
	for (page = 0xb0; page < 0xb8; page++)
  	{
	    oled_write_operate(OLED_COMM, page);
	    oled_write_operate(OLED_COMM, 0x10);
	    oled_write_operate(OLED_COMM, 0x00);
		
		for (seg = 0; seg < 128; seg++)
		{
			oled_write_operate(OLED_DATA, *pdata++);
		}

//		if (page >= 0xB2 && page <= 0xB5)	//����ͼƬ����
//		{
//			for (seg = 0; seg < 128; seg++)
//			{
//				oled_write_operate(OLED_DATA, *pdata++);
//			}
//		}
//		else
//		{
//			for (seg = 0; seg < 128; seg++)
//			{
//				oled_write_operate(OLED_DATA, 0x00);
//			}
//		}
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_pow
//	����˵��: ���� m��n�η�
//	��    ��: m������;n��ָ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint32_t oled_pow(uint8_t m, uint8_t n)
{
	uint32_t ret = 1;
	
	while(n--)
	{
		ret *= m;
	}
	
	return ret;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_dis_picture
//	����˵��: oled ����
//	��    ��: x��������(��);y:������(��);num����ʾ����;len�����ֳ���;size_num�����ִ�С;dis_mode����ʾ����
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: ��
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_dis_num(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size_num, uint8_t dis_mode)
{
	uint8_t i = 0;
	uint8_t temp = 0;
	uint8_t enshow = 0;
	
	for (i = 0; i < len; i++)
	{
		temp = (num / (oled_pow(10,len-i-1))) % 10;//����ʾ������һλһλȡ����
		if((enshow == 0) && (i < (len-1)))
		{
			if(temp == 0)
			{
				if (dis_mode&0x80)
					oled_dis_one_char(x + (size_num / 2) * i, y, '0');
				else
					oled_dis_one_char(x + (size_num / 2) * i, y, ' ');
				
				continue;
			}
			else
			{
				enshow = 1;
			}
		}
		oled_dis_one_char(x + (size_num / 2) * i, y, temp + '0');
	}
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	�� �� ��: oled_init
//	����˵��: oled ��ʼ��
//	��    ��: ��
//	�� �� ֵ: ��
//	��    ��: 2020-04-11
//  ��    ע: �ⲿ���ó�ʼ��OLED
//	��    ��: by ����AI
//---------------------------------------------------------------------------------------------------------------------------------------------
void oled_init(void)
{
	oled_gpio_init();	//oled cs/dc/rst ����io��ʼ��
	
	OLED_RST_HIGH;
	delay_ms(10);
	OLED_RST_LOW;
	delay_ms(10);
	OLED_RST_HIGH; 
		
	oled_write_operate(OLED_COMM, 0xAE);	//�ر�OLED
	oled_write_operate(OLED_COMM, 0x00);//�����е�λ��ַ
	oled_write_operate(OLED_COMM, 0x10);//�����и�λ��ַ
	oled_write_operate(OLED_COMM, 0x40);	//������ʼ�е�ַ��ӳ��RAM��ʾ��ʼ�� (0x00~0x3F)
	oled_write_operate(OLED_COMM, 0x81);	//�Աȶ�����
	oled_write_operate(OLED_COMM, 0xCF); 	// Set SEG Output Current Brightness
	oled_write_operate(OLED_COMM, 0xA1);	//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
	oled_write_operate(OLED_COMM, 0xC8);	//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
	oled_write_operate(OLED_COMM, 0xA8);	//��������·��(1 to 64)
	oled_write_operate(OLED_COMM, 0x3f);	//--1/64 duty
	oled_write_operate(OLED_COMM, 0xD3);	//-������ʾƫ��(0x00~0x3F)
	oled_write_operate(OLED_COMM, 0x00);	//-not offset
	oled_write_operate(OLED_COMM, 0xd5);	//--set display clock divide ratio/oscillator frequency
	oled_write_operate(OLED_COMM, 0x80);	//--set divide ratio, Set Clock as 100 Frames/Sec
	oled_write_operate(OLED_COMM, 0xD9);	//--set pre-charge period
	oled_write_operate(OLED_COMM, 0xF1);	//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	oled_write_operate(OLED_COMM, 0xDA);	//--set com pins hardware configuration
	oled_write_operate(OLED_COMM, 0x12);
	oled_write_operate(OLED_COMM, 0xDB);	//--set vcomh
	oled_write_operate(OLED_COMM, 0x40);	//Set VCOM Deselect Level
	oled_write_operate(OLED_COMM, 0x20);	//����ҳ��ַģʽ(0x00/0x01/0x02)
	oled_write_operate(OLED_COMM, 0x02);	//
	oled_write_operate(OLED_COMM, 0x8D);	//--set Charge Pump enable/disable
	oled_write_operate(OLED_COMM, 0x14);	//--set(0x10) disable
	oled_write_operate(OLED_COMM, 0xA4);	//��ʾ����(��ʾ:A4;����ʾ:A5)
	oled_write_operate(OLED_COMM, 0xA7);	// ������������ʾ (0xa6:����;a7:����) 
	oled_write_operate(OLED_COMM, 0xAF);	//����ʾ
	
	oled_write_operate(OLED_COMM, 0xAF); 	//display ON(on:AF;off:AE)
	oled_dis_clear();
	oled_set_pos(0, 0); 	
}  

#endif /* __PRJ_STM32F10X_DRVOLED_C__ */
