//---------------------------------------------------------------------------------------------------------------------------------------------
//平    台:				STM32F10X
//文    件:    		 	drultrasonic.c
//作    者:       		霁风AI
//库版 本:   			Vxxx
//文件版本:   			V1.0.0
//日   期:      		2021年04月11日
//说   明:      	 	超声波模块 HC-SR04 驱动实现
//----------------------------------------------------------------------------------------------------------------------------------------------

#ifndef __PRJ_STM32F40X_DRVMPU6050_C__
#define __PRJ_STM32F40X_DRVMPU6050_C__

#include "drvmpu6050.h"
#include "delay.h"
#include "drvuart.h"
#include "drvsfi2c.h"
#include "stdio.h"

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_set_gyro_fsr
//	功能说明: mpu6050陀螺仪满量程范围设置
//	形    参: 	fsr_val：0:±250dps;1:±500dps;2:±1000dps;3:±2000dps
//	返 回 值: 0：成功;其他：失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_set_gyro_fsr(uint8_t fsr_val)
{
	uint8_t ret_val;
	
	ret_val = mpu6050_write_byte(MPU_GYRO_CFG_REG, fsr_val<<3);	//设置陀螺仪满量程范围
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_set_accel_fsr
//	功能说明: mpu6050加速度满量程范围设置
//	形    参: 	fsr_val：0:±2g;1:±4g;2:±8g;3:±16g
//	返 回 值: 0：成功;其他：失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_set_accel_fsr(uint8_t fsr_val)
{
	uint8_t ret_val;
	
	ret_val = mpu6050_write_byte(MPU_ACCEL_CFG_REG, fsr_val<<3);	//设置加速度满量程范围
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_set_lpf
//	功能说明: mpu6050低通数字滤波器设置
//	形    参: 	lpf_val：数字低通滤波器的频率(Hz)
//	返 回 值: 0：成功;其他：失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_set_lpf(uint16_t lpf_val)
{
	uint8_t ret_val, temp = 0;
	
	if (lpf_val >= 188)
		temp = 1;
	else if (lpf_val >= 98)
		temp = 2;
	else if (lpf_val >= 42)
		temp = 3;
	else if (lpf_val >= 20)
		temp = 4;
	else if (lpf_val >= 10)
		temp = 5;
	else 
		temp = 6;
	
	ret_val = mpu6050_write_byte(MPU_CFG_REG, temp);	//设置数字低通滤波器
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_set_rate
//	功能说明: mpu6050采样率设置
//	形    参: 	rate_val：采样频率(Hz)
//	返 回 值: 0：成功;其他：失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_set_rate(uint16_t rate_val)
{
	uint8_t dat, ret_val;
	
	if (rate_val > 1000)
		rate_val = 1000;
	if (rate_val < 4)
		rate_val = 4;
	
	dat = 1000 / rate_val - 1;
	dat = mpu6050_write_byte(MPU_SAMPLE_RATE_REG, dat);	//设置数字低通滤波器
	
	ret_val = mpu6050_set_lpf(rate_val/2);	//LPF为采样率的一半
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_read_dev_id
//	功能说明: mpu6050读取设备ID
//	形    参: 无
//	返 回 值: id_val：ID(0x68)
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t mpu6050_read_dev_id(void)
{
	uint8_t id_val;
	
	id_val = mpu6050_read_byte(MPU_DEVICE_ID_REG);	//读取器件ID(0x68)
	
	return id_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_get_temp
//	功能说明: mpu6050读取温度值
//	形    参: 无
//	返 回 值: temp：温度值
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
float mpu6050_get_temp(void)
{
	uint8_t buff[2];
	volatile int16_t temp;
	volatile float ret_val;
	
	mpu6050_read_data(MPU_ADDR, MPU_TEMP_OUTH_REG, buff, 2);
	temp = (uint16_t)((buff[0]<<8) | buff[1]);
	ret_val = 36.53 + ((double)temp) / 340;	
//	printf("temp : %f \r\n", temp);
//	temp *= 100;	//温度值扩大100倍
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_get_gyro_data
//	功能说明: mpu6050读取陀螺仪原始值
//	形    参: 	*gx：x轴原始数据
//				*gy：y轴原始数据
//				*gz：z轴原始数据
//	返 回 值: ret_val：0-成功;其他代码-失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_get_gyro_data(int16_t *gx, int16_t *gy, int16_t *gz)
{
	uint8_t buff[6], ret_val;
	
	ret_val = mpu6050_read_data(MPU_ADDR, MPU_GYRO_XOUTH_REG, buff, 6);
	if (ret_val == 0)
	{
		*gx = (uint16_t)((buff[0] << 8) | buff[1]);
		*gy = (uint16_t)((buff[2] << 8) | buff[3]);
		*gz = (uint16_t)((buff[4] << 8) | buff[5]);
	}
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_get_accel_data
//	功能说明: mpu6050读取加速度原始值
//	形    参: 	*ax：x轴原始数据
//				*ay：y轴原始数据
//				*az：z轴原始数据
//	返 回 值: ret_val：0-成功;其他代码-失败
//  备    注: 
//	日    期: 2021-04-30
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_get_accel_data(int16_t *ax, int16_t *ay, int16_t *az)
{
	uint8_t buff[6], ret_val;
	
	ret_val = mpu6050_read_data(MPU_ADDR, MPU_ACCEL_XOUTH_REG, buff, 6);
	if (ret_val == 0)
	{
		*ax = (uint16_t)((buff[0] << 8) | buff[1]);
		*ay = (uint16_t)((buff[2] << 8) | buff[3]);
		*az = (uint16_t)((buff[4] << 8) | buff[5]);
	}
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_init
//	功能说明: mpu6050 初始化设置
//	形    参: 无
//	返 回 值: 无
//  备    注: 1：失败；0：成功
//	日    期: 2021-04-11
//	作    者: by 霁风AI
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_init(void)
{
	uint8_t ret_val;
	
	I2c_Init();	//I2C 模拟总线初始化
	
	mpu6050_write_byte(MPU_PWR_MGMT1_REG, 0x80);	//复位 mpu6050
	delay_ms(100);
	mpu6050_write_byte(MPU_PWR_MGMT1_REG, 0x00);	//唤醒mpu6050
	mpu6050_set_gyro_fsr(3);	//陀螺仪:±2000dps
	mpu6050_set_accel_fsr(0);	//加速度:±2g
	mpu6050_set_rate(50);	//采样率:50Hz
	
	mpu6050_write_byte(MPU_INT_EN_REG, 0x00);	//关闭所有中断
	mpu6050_write_byte(MPU_USER_CTRL_REG, 0x00);	//I2C主模式关闭
	mpu6050_write_byte(MPU_FIFO_EN_REG, 0x00);	//关闭FIFO
	mpu6050_write_byte(MPU_INTBP_CFG_REG, 0x80);	//INT 引脚低电平有效
	
	ret_val = mpu6050_read_dev_id();	//读取器件ID(0x68)
	if (ret_val == MPU_ADDR)
	{
		mpu6050_write_byte(MPU_PWR_MGMT1_REG, 0x01);	//设置CKLSEL,PLL 以x轴为参考
		mpu6050_write_byte(MPU_PWR_MGMT2_REG, 0x00);	//加速度和陀螺仪均启动
		mpu6050_set_rate(50);
//		printf("mpu init sucessed! \r\n");
	}
	else 
	{
//		printf("mpu init failed! \r\n");
		return 1;
	}
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_write_byte
//	功能说明: mpu6050向寄存器写入数值
//	形    参: 	reg_addr：寄存器地址
//				reg_val：写入的数值
//	返 回 值: 1：失败；0：成功
//  备    注: I2C 的主机——>从机时序
//	日    期: 2021-04-27
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_write_byte(uint8_t reg_addr, uint8_t reg_val)
{
	I2c_Start();
	I2c_SendOneByte((MPU_ADDR<<1) & 0xfe);	//i2C发送数据,写入地址最低位为0
	if (I2c_WaitAckDelay(250))
	{
		I2c_Stop();
		return 1;
	}
	
	I2c_SendOneByte(reg_addr);	//写入寄存器地址
	I2c_WaitAckDelay(250);
	I2c_SendOneByte(reg_val);	//写入寄存器数据
	if (I2c_WaitAckDelay(250))
	{
		I2c_Stop();
		return 1;
	}
	
	I2c_Stop();
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_read_byte
//	功能说明: mpu6050向寄存器读取数值
//	形    参: 	reg_addr：寄存器地址
//	返 回 值: ret_val：返回的数值
//  备    注: I2C 的主机——>从机时序
//	日    期: 2021-04-28
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_read_byte(uint8_t reg_addr)
{
	uint8_t ret_val;
	
	//1.先写入地址
	I2c_Start();
	I2c_SendOneByte((MPU_ADDR<<1) & 0xfe);
	I2c_WaitAckDelay(250);
	I2c_SendOneByte(reg_addr);
	I2c_WaitAckDelay(250);
	
	//2.读取数值
	I2c_Start();
	I2c_SendOneByte((MPU_ADDR<<1) | 0x01);	//发送从设备地址 + 读命令
	I2c_WaitAckDelay(250);
	ret_val = I2c_RecvOneByte(1);	//读取数据 + 发送NACK
	I2c_Stop();
	
	return ret_val;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_write_data
//	功能说明: mpu6050向寄存器写入一串数据
//	形    参: 	dev_addr：从设备地址(写数据)
//				reg_addr：寄存器地址
//				dat_buf：写入的数值
//				dat_len：写入数据的长度
//	返 回 值: 1：失败；0：成功
//  备    注: I2C 的主机——>从机时序
//	日    期: 2021-04-29
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_write_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t *dat_buf, uint16_t dat_len)
{
	uint16_t i;
	
	I2c_Start();
//	I2c_SendOneByte(dev_addr);	//发送I2C从设备地址 + 写命令
	I2c_SendOneByte((dev_addr<<1) & 0xfe);
	if (I2c_WaitAckDelay(250))
	{
		I2c_Stop();
		return 1;
	}
	
	I2c_SendOneByte(reg_addr);	//操作具体的寄存器地址
	I2c_WaitAckDelay(250);
	for (i = 0; i < dat_len; i++)
	{
		I2c_SendOneByte(dat_buf[i]);
		if (I2c_WaitAckDelay(250))
		{
			I2c_Stop();
			return 1;
		}
	}
	
	I2c_Stop();
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_read_data
//	功能说明: mpu6050向寄存器读取一串数据
//	形    参: 	dev_addr：从设备地址(写数据)
//				reg_addr：寄存器地址
//				dat_buf：写入的数值
//				dat_len：写入数据的长度
//	返 回 值: 1：失败；0：成功
//  备    注: I2C 的主机——>从机时序
//	日    期: 2021-04-29
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t mpu6050_read_data(uint8_t dev_addr, uint8_t reg_addr, uint8_t *dat_buf, uint16_t dat_len)
{
	uint16_t i;
	
	I2c_Start();	
	I2c_SendOneByte((dev_addr<<1) & 0xfe);	//发送从设备地址 + 写命令
	if (I2c_WaitAckDelay(250))
	{
		I2c_Stop();
		return 1;
	}
	
	I2c_SendOneByte(reg_addr);	//发送寄存器地址
	I2c_WaitAckDelay(250);
	
//	dev_addr = MPU_ADDR_RD;	//修改为读
	I2c_Start();	//开始读数据时序
//	I2c_SendOneByte(dev_addr);	//发送从设备地址 + 读命令
	I2c_SendOneByte((dev_addr<<1) | 0x01);
	I2c_WaitAckDelay(250);
	for (i = 0; i < dat_len; i++)
	{
		if (i == (dat_len-1))
		{
			dat_buf[i] = I2c_RecvOneByte(1);	//最后一个数据发送NACK
		}
		else 
		{
			dat_buf[i] = I2c_RecvOneByte(0);	//其余数据发送应答ACK
		}
	}
	
	I2c_Stop();
	return 0;
}

//为了移植 dmp 处理代码
uint8_t hal_i2c_write_data(uint8_t dev_addr, uint8_t reg_addr, uint16_t dat_len, uint8_t *dat_buf)
{
	return mpu6050_write_data(dev_addr, reg_addr, dat_buf, dat_len);
}

uint8_t hal_i2c_read_data(uint8_t dev_addr, uint8_t reg_addr, uint16_t dat_len, uint8_t *dat_buf)
{
	return mpu6050_read_data(dev_addr, reg_addr, dat_buf, dat_len);
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: usart_report_data
//	功能说明: 串口上传数据到[匿名上位机], 按照一定格式组装数据
//	形    参: 	func：功能选择(0xa0 -- 0xaf)
//				send_dat：发送的数值(数据最大28byte)
//				dat_len：发送数据的长度
//	返 回 值: 1：失败；0：成功
//  备    注: 匹配匿名上位机
//	日    期: 2021-05-19
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
uint8_t usart_report_data(uint8_t func, uint8_t *send_dat, uint8_t dat_len)
{
	uint8_t send_buf[32];
	uint8_t i ;
	
	if (dat_len > 28)	//每次发送的数据不能超过28字节
		return 1;
	
	send_buf[dat_len+3] = 0x00;	//参与校验部分的数据先清0(头部3字节+实际数据)
	send_buf[0] = 0x88;	//帧头
	send_buf[1] = func;	//功能字
	send_buf[2] = dat_len;	//数据常数
	
	for (i=0; i<dat_len; i++)
	{
		send_buf[i+3] = send_dat[i];	//数据重组
	}
	for (i=0; i<dat_len+3; i++)
	{
		send_buf[dat_len+3] += send_buf[i];	//计算校验和
	}
	for (i=0; i<dat_len+4; i++)
	{
		uart_send_byte(1, send_buf[i]);	//发送数据到串口1
	}
	return 0;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: mpu6050_send_data_report
//	功能说明: mpu6050通过串口向上位机发送帧格式数据
//	形    参: 	a_x/a_y/a_z：加速度 xyz 三轴数据
//				g_x/g_y/g_z：陀螺仪 xyz 三轴数据
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-05-19
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
void mpu6050_send_data_report(int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z)
{
	uint8_t tx_buf[12];
	
	tx_buf[0] = (a_x>>8) & 0xff;	//高位先发送
	tx_buf[1] = (a_x>>0) & 0xff;	//低位后发送
	tx_buf[2] = (a_y>>8) & 0xff;
	tx_buf[3] = (a_y>>0) & 0xff;
	tx_buf[4] = (a_z>>8) & 0xff;
	tx_buf[5] = (a_z>>0) & 0xff;
	tx_buf[6] = (g_x>>8) & 0xff;
	tx_buf[7] = (g_x>>0) & 0xff;
	tx_buf[8] = (g_y>>8) & 0xff;
	tx_buf[9] = (g_y>>0) & 0xff;
	tx_buf[10] = (g_z>>8) & 0xff;
	tx_buf[11] = (g_z>>0) & 0xff;
	
	usart_report_data(0xA1, tx_buf, 12);	//帧数据: 0xA1
}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	函 数 名: usart_report_imu_data
//	功能说明: 通过串口向上位机发送六轴姿态数据
//	形    参: 	a_x/a_y/a_z：加速度 xyz 三轴数据
//				g_x/g_y/g_z：陀螺仪 xyz 三轴数据
//				roll：横滚角(单位：0.01° -18000~18000 <==> -180°~180°)
//				pitch：俯仰角(单位：0.01° -9000~9000 <==> -90°~90°)
//				yaw：偏航角(单位：0.1° 0~360 <==> 0°~360°)
//	返 回 值: 无
//  备    注: 
//	日    期: 2021-05-19
//	作    者: by 霁风
//---------------------------------------------------------------------------------------------------------------------------------------------
void usart_report_imu_data(int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z, int16_t roll, int16_t pitch, int16_t yaw)
{
	uint8_t tx_buf[28];
	uint8_t i;
	
	for (i=0; i<28; i++)
	{
		tx_buf[i] = 0x00;
	}
	
	tx_buf[0] = (a_x>>8) & 0xff;	//高位先发送
	tx_buf[1] = (a_x>>0) & 0xff;	//低位后发送
	tx_buf[2] = (a_y>>8) & 0xff;
	tx_buf[3] = (a_y>>0) & 0xff;
	tx_buf[4] = (a_z>>8) & 0xff;
	tx_buf[5] = (a_z>>0) & 0xff;
	tx_buf[6] = (g_x>>8) & 0xff;
	tx_buf[7] = (g_x>>0) & 0xff;
	tx_buf[8] = (g_y>>8) & 0xff;
	tx_buf[9] = (g_y>>0) & 0xff;
	tx_buf[10] = (g_z>>8) & 0xff;
	tx_buf[11] = (g_z>>0) & 0xff;
	
	tx_buf[18] = (roll>>8) & 0xff;
	tx_buf[19] = (roll>>0) & 0xff;
	tx_buf[20] = (pitch>>8) & 0xff;
	tx_buf[21] = (pitch>>0) & 0xff;
	tx_buf[22] = (yaw>>8) & 0xff;
	tx_buf[23] = (yaw>>0) & 0xff;
	
	usart_report_data(0xAF, tx_buf, 28);	//飞控显示帧
}

#endif	//__PRJ_STM32F40X_DRVMPU6050_C__
