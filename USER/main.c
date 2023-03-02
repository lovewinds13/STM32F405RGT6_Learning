#include "sys.h"
#include "delay.h"
#include "drvled.h"
#include "drvuart.h"
#include "hardware_spi.h"
#include "drvoled.h"
#include "drvexflash.h"
#include "drvtimer.h"
#include "drvsfi2c.h"
#include "app_htu21d.h"
#include "drvmpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "drvultrasonic.h"
#include "drvpwm.h"
#include "drvadc.h"
#include "drvblehc05.h"
#include "drvesp8266.h"
#include "drvbutton.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

extern BLE_HC05_T g_t_ble;
extern WIFI_ESP_T g_t_wifi;

int func_test(void)
{
	uint8_t len;

	len = strlen("ABC");
	
	return len;
}
//串口测试
int main_uart(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);		  //初始化延时函数
//	Bsp_LedInit();		        //初始化LED端口
	uart_init(1, 9600);
	
	printf("start! \r\n");	
	
	while(1)
	{
//		Bsp_LedToggle(0);
//		delay_ms(50);
//		uart_test();
//		uart_send_serial_bytes(2, "error", 6);

	}
}

//OLED测量
int main_oled(void)
{
	delay_init(168);
	Bsp_LedInit();
	spi_master_init(1);
	oled_init();
	oled_dis_logo();
	
	while (1)
	{	
		Bsp_LedToggle(1);
		delay_ms(200);
	}
}

//SPI FLASH测试
int main_xflash(void)
{
	uint32_t ulje_id;
	uint8_t tm_buf[3] = {11, 22, 44};
//	uint8_t tm_buf[4] = {'A', 'B', 'C'};
	
	delay_init(168);
	Bsp_LedInit();
	spi_flash_gpio_init();
	spi_master_init(1);
	oled_init();
	
	ulje_id = Flash_ReadJEDECID();
	
	Flash_EraseSector(0);
	Flash_WriteNoCheck(tm_buf, 0x00, 3);
	delay_ms(2);
	memset(tm_buf, 0, 3);
	Flash_ReadSomeBytes(tm_buf, 0x00, 3);
	
	oled_dis_num(4, 4, 77, 2, 16, 0x80);
	oled_dis_num(50, 4, tm_buf[1], 2, 16, 0x80);
	oled_dis_num(100, 4, tm_buf[2], 2, 16, 0x80);
	
	while(1) 
	{
		Bsp_LedToggle(1);
		delay_ms(200);
	}
}
//定时器测量
int main_timer(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	timer_init(3);
	
	while (1)
	{	
		
	}
}

//HTU21D 测量
int main_htu(void)
{
	volatile float ret_val;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	uart_init(1, 9600);
	Htu_Init();
	
	printf("start! \r\n");
	
	while (1)
	{	
		Htu_Display();
		
		Bsp_LedToggle(1);
		delay_ms(1000);
	}
}

//mpu6050 测试
#define MPU_DMP	1
int main_mpu6050(void)
{
	float ret_val;
	int16_t gyro_buff[3];
	int16_t accel_buff[3];
	float pitch, roll, yaw;
	int16_t temp;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	uart_init(1, 500000);
	spi_master_init(1);
	oled_init();
	mpu6050_init();
	
	#if MPU_DMP
	oled_dis_str(2, 0, " Temp:    . C");
	oled_dis_str(2, 2, "Pitch:    . C");	
 	oled_dis_str(2, 4, " Roll:    . C");	 
 	oled_dis_str(2, 6, " Yaw :    . C");
	
	while (mpu_dmp_init())
	{
		oled_dis_str(2, 4, "mpu init failed");
	}
//	printf ("mpu6050 dmp init sucessed! \r\n");
	#endif
	
	while (1)
	{	
	#if MPU_DMP
//			ret_val = mpu6050_get_temp();
//			printf("mpu6050 temperature measure value is %f ℃\r\n", ret_val);
		
		if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)	//延时太大 fifo 会溢出
		{
			temp = pitch * 10;	//放大10倍,方便液晶显示处理
			if (temp < 0)
			{
				oled_dis_one_char(2+48, 2, '-');
				temp = -temp;
			}
			else 
			{
				oled_dis_one_char(2+48, 2, ' ');
			}
			oled_dis_num(2+48+8, 2, temp/10, 3, 16, 0x80);	//整数部分
			oled_dis_num(2+48+40, 2, temp%10, 1, 16, 0x80);	//小数部分
			
			temp = roll * 10;
			if (temp < 0)
			{
				oled_dis_one_char(2+48, 4, '-');
				temp = -temp;
			}
			else 
			{
				oled_dis_one_char(2+48, 4, ' ');
			}
			oled_dis_num(2+48+8, 4, temp/10, 3, 16, 0x80);	//整数部分
			oled_dis_num(2+48+40, 4, temp%10, 1, 16, 0x80);	//小数部分
			
			temp = yaw * 10;
			if (temp < 0)
			{
				oled_dis_one_char(2+48, 6, '-');
				temp = -temp;
			}
			else 
			{
				oled_dis_one_char(2+48, 6, ' ');
			}
			oled_dis_num(2+48+8, 6, temp/10, 3, 16, 0x80);	//整数部分
			oled_dis_num(2+48+40, 6, temp%10, 1, 16, 0x80);	//小数部分
//			
			ret_val = mpu6050_get_temp();
//			printf("mpu6050 temperature measure value is %f ℃\r\n", ret_val);
			temp = ret_val * 100;
			if (temp < 0)
			{
				oled_dis_one_char(2+48, 0, '-');
				temp = -temp;
			}
			else 
			{
				oled_dis_one_char(2+48, 0, ' ');
			}
			oled_dis_num(2+48+8, 0, temp/100, 3, 16, 0x80);	//整数部分
			oled_dis_num(2+48+40, 0, temp%10, 1, 16, 0x80);	//小数部分
//			
			mpu6050_get_gyro_data(&gyro_buff[0], &gyro_buff[1], &gyro_buff[2]);
//			printf("**gyro data are : %d  %d  %d  \r\n", gyro_buff[0], gyro_buff[1], gyro_buff[2]);
			
			mpu6050_get_accel_data(&accel_buff[0], &accel_buff[1], &accel_buff[2]);
//			printf("##accel_buff data are : %d  %d  %d  \r\n", accel_buff[0], accel_buff[1], accel_buff[2]);
			
//			printf("**sensor data are : %4f  %4f  %4f  \r\n", pitch, roll, yaw);
			
			mpu6050_send_data_report(accel_buff[0], accel_buff[1], accel_buff[2], gyro_buff[0], gyro_buff[1], gyro_buff[2]);
			usart_report_imu_data(accel_buff[0], accel_buff[1], accel_buff[2], gyro_buff[0], gyro_buff[1], gyro_buff[2], (int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		
			delay_ms(50);
		}
		
	#else
		ret_val = mpu6050_get_temp();
		printf("mpu6050 temperature measure value is %f ℃\r\n", ret_val);
		
//		mpu6050_get_gyro_data(&gyro_buff[0], &gyro_buff[1], &gyro_buff[2]);
//		printf("**gyro data are : %d  %d  %d  \r\n", gyro_buff[0], gyro_buff[1], gyro_buff[2]);
//		
//		mpu6050_get_accel_data(&accel_buff[0], &accel_buff[1], &accel_buff[2]);
//		printf("##accel_buff data are : %d  %d  %d  \r\n", accel_buff[0], accel_buff[1], accel_buff[2]);
	#endif
//		Bsp_LedToggle(0);
//		delay_ms(20);
//		Bsp_LedToggle(1);
//		delay_ms(20);
		delay_ms(100);
	}
}

//超声波测量
int main_hcsr04(void)
{
	float ret_val;
	
	Bsp_LedInit();
	hcsr04_gpio_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	uart_init(1, 9600);
	
	while (1)
	{	
		ret_val = hcsr04_measure();
		printf("hcsr04 measure value is %f cm\r\n", ret_val);
		
		Bsp_LedToggle(0);
		delay_ms(200);
		Bsp_LedToggle(1);
		delay_ms(200);
	}
}

//pwm测试
int main_pwm(void)
{
	uint16_t tim_cnt = 0;
	uint8_t flag = 1;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	uart_init(1, 9600);
	pwm_init(1);
	
	while (1)
	{	
		delay_ms(5);
		
		if (flag)
		{
			tim_cnt++;
		}
		else 
		{
			tim_cnt--;
		}
		if (tim_cnt > 100)
		{
			flag = 0;
		}
		if (tim_cnt == 0)
		{
			flag = 1;
		}
		
		TIM_SetCompare2(TIM2, tim_cnt);
	}
}

//ADC 测试
int main_adc(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	uart_init(1, 9600);
	adc_init();
	spi_master_init(1);
	oled_init();
	oled_dis_logo();
	delay_ms(3000);
	oled_dis_clear();
	oled_dis_str(2, 2, "adc_val:");
	oled_dis_str(2, 4, "adc_vol:0.000v");
	
	printf("start! \r\n");
	
	while (1)
	{	
		adc_test();
		
		Bsp_LedToggle(1);
		delay_ms(200);
	}
}

//ble测试
int main_ble(void)
{
	char tmp_buf[10];
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	uart_init(DEBUG_COMM_NO, 9600);
	ble_init();
	spi_master_init(1);
	oled_init();
	oled_dis_logo();
	delay_ms(3000);
	oled_dis_clear();
	
	printf("start! \r\n");
	
	ble_detect();
	ble_cmd_test();
	
	while (1)
	{	
		Bsp_LedToggle(1);
		delay_ms(100);	
			
		if (g_t_ble.ble_rx_buff[0] == 0x55)
		{
			g_t_ble.ble_rx_buff[0] = 0xBB;
			oled_dis_clear();
			oled_dis_str(2, 2, "ble recv ok!");
			uart_send_serial_bytes(BLE_COMM_NO, g_t_ble.ble_rx_buff, 1);	//发送数据,APP端可见
//			uart_send_serial_bytes(DEBUG_COMM_NO, g_t_ble.ble_rx_buff, 1);
			
			ble_reset_ref();
		}
		if (g_t_ble.ble_rx_buff[0] == 0xAA)
		{
			sprintf(tmp_buf, "%02X", g_t_ble.ble_rx_buff[0]);
			oled_dis_clear();
			oled_dis_str(2, 2, (uint8_t *)tmp_buf);
			
			uart_send_serial_bytes(BLE_COMM_NO, g_t_ble.ble_rx_buff, 1);
//			uart_send_serial_bytes(DEBUG_COMM_NO, g_t_ble.ble_rx_buff, 1);
			
			ble_reset_ref();
		}
		if (strcmp((const char*)g_t_ble.ble_rx_buff, "CH") == 0)
		{
			oled_dis_clear();
			oled_dis_str(2, 2, "str ok!");
			
			uart_send_serial_bytes(BLE_COMM_NO, g_t_ble.ble_rx_buff, 2);
			uart_send_serial_bytes(DEBUG_COMM_NO, g_t_ble.ble_rx_buff, 2);
			
			ble_reset_ref();
		}
	}
}

//button测试
int main_btn(void)
{
	uint8_t btn_val = 0;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	Bsp_LedInit();
	button_gpio_init();
	uart_init(DEBUG_COMM_NO, 9600);
	spi_master_init(1);
	oled_init();
	oled_dis_logo();
	delay_ms(3000);
	oled_dis_clear();
	
	printf("start! \r\n");
	while (1)
	{	
		Bsp_LedToggle(1);
		delay_ms(100);
		
		btn_val = button_detect(0);
		
//		if (btn_val == KEY1_DOWN)
//		{
//			oled_dis_clear();
//			oled_dis_str(2, 0, "key1 ok");
//		}
//		if (btn_val == KEY2_DOWN)
//		{
//			oled_dis_clear();
//			oled_dis_str(2, 0, "key2 ok");
//		}
		
		switch (btn_val)
		{
			case KEY1_DOWN:
				oled_dis_clear();
				oled_dis_str(2, 0, "key1 ok");
			break;
			case KEY2_DOWN:
				oled_dis_clear();
				oled_dis_str(2, 4, "key2 ok");
			break;
			default:
				oled_dis_str(2, 2, "no key");
			break;
		}
	}
}

//ESP8266
int main_esp8266(void)
{
	uint16_t i;
	uint8_t tmp_buf[] = "esp8266 test";
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	// 设置中断优先级分组2
	delay_init(168);
	
	Bsp_LedInit();
	uart_init(DEBUG_COMM_NO, 9600);
	
	spi_master_init(1);
	oled_init();
	oled_dis_logo();
	delay_ms(2000);
	oled_dis_clear();
	
	printf("start! \r\n");
	
	esp8266_init();
	esp8266_test();
	printf("esp8266 is running! \r\n");
	esp8266_uart_trans(1);
	
	printf("esp8266 start sta mode! \r\n");
	
	while (1)
	{
		Bsp_LedToggle(1);
		delay_ms(100);
		
		if ((g_t_wifi.wifi_rx_buff[0] == 0x55) && (g_t_wifi.wifi_rx_buff[1] == 0xAA) && (g_t_wifi.wifi_comm_mode == ESP_RX_OK))
		{
//			for (i = 0; i < g_t_wifi.wifi_rx_length; i++)
			{
				oled_dis_str(2, 4, "wifi receive");
			}
			uart_send_serial_bytes(ESP8266_COMM_NO, tmp_buf, sizeof(tmp_buf));
			delay_ms(500);
			esp8266_reset_ref();
		}
		else
		{
			oled_dis_str(2, 2, "wifi free");
		}
		
	}
}
	
int main(void)
{
	main_esp8266();
}
