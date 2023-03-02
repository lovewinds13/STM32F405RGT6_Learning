#ifndef __HARDWARE_SPI__H__
#define __HARDWARE_SPI__H__

#include "sys.h"

//#define SPI1_IRQ_ENABLE
#define SPIM1_GPIO_PORT	    GPIOA

#define SPIM1_CLK_IO	(GPIO_Pin_5)
#define SPIM1_MISO_IO	(GPIO_Pin_6)
#define SPIM1_MOSI_IO	(GPIO_Pin_7)

#define RCC_PCLK_SPIM1_GPIO     RCC_AHB1Periph_GPIOA
#define RCC_PCLK_SPIM1_HD       RCC_APB2Periph_SPI1


extern void spi_master_init(uint8_t spi_chl);			 //��ʼ��SPI��
extern uint8_t spi_master_send_recv_byte(uint8_t spi_chl, uint8_t spi_byte);
extern void spi_master_nvic_set(void);

extern void spi_master_speed_set(uint8_t spi_chl, uint8_t spi_speed); //����SPI�ٶ�   
extern uint8_t spi_master_send_byte(uint8_t spi_chl, uint8_t send_byte);//SPI���߶�дһ���ֽ�
extern uint8_t spi_master_recv_byte(uint8_t spi_chl);

extern void spi_master_send_some_bytes(uint8_t spi_chl, uint8_t *pbdata, uint16_t send_length);
extern void spi_master_recv_some_bytes(uint8_t spi_chl, uint8_t *pbdata, uint16_t recv_length);

//u8 SPI1_ReadWriteByte(u8 TxData);
uint8_t SPI_WriteReadByte(uint8_t TxData);
uint16_t SFLASH_ReadID(void);
uint16_t SPI_Flash_ReadID(void);
uint8_t SPI1_ReadWriteByte(uint8_t TxData);

#endif	//__HARDWARE_SPI__H__

